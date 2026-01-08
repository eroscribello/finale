

#include "kdl_control.h"
#include <stdio.h>
#include <iostream>
#include <sstream> // Necessario per std::stringstream
#include <cstdlib>
#include "rclcpp/rclcpp.hpp"

KDLController::KDLController(){}

KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis() + robot_->getGravity() /*friction compensation?*/;
}

/* Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)
{

} */



KDL::JntArray KDLController::velocity_ctrl_null(Eigen::Matrix<double,6,1> error_position, int Kp)
{
    unsigned int nj = robot_->getNrJnts();
    Eigen::MatrixXd J = robot_->getEEJacobian().data;
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::MatrixXd JntLimits_ = robot_->getJntLimits();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(nj, nj);

    // --- 1. PSEUDO-INVERSA ROBUSTA (Damped Least Squares) ---
    // Impedisce velocità infinite vicino alle singolarità
    double damping = 0.01;
    Eigen::MatrixXd Jt = J.transpose();
    Eigen::MatrixXd J_pinv = Jt * (J * Jt + damping * damping * Eigen::MatrixXd::Identity(6, 6)).inverse();

    // --- 2. NULL SPACE VELOCITY (Seconda priorità) ---
    // Semplificato: spinge i giunti verso il centro del loro range senza divisioni pericolose
    Eigen::VectorXd q0_dot(nj);
    double gain_null = 1.0; 
    for (unsigned int i = 0; i < nj; i++) {
        double q_mid = (JntLimits_(i, 1) + JntLimits_(i, 0)) / 2.0;
        q0_dot(i) = gain_null * (q_mid - q(i)); 
    }

    // --- 3. CALCOLO VELOCITÀ FINALE ---
    // Se continua a divergere, cambia il segno di (Kp * error_position) in (-Kp * error_position)
    Eigen::VectorXd qd_vec = J_pinv * (double(Kp) * error_position) + (I - J_pinv * J) * q0_dot;

    // --- 4. CLAMPING DI SICUREZZA ---
    // Impedisce al robot di muoversi troppo velocemente (max 1.0 rad/s)
    for (unsigned int i = 0; i < nj; i++) {
        if (qd_vec(i) > 1.0) qd_vec(i) = 1.0;
        if (qd_vec(i) < -1.0) qd_vec(i) = -1.0;
    }

    KDL::JntArray qd(nj);
    qd.data = qd_vec;
    return qd;
}





KDL::JntArray KDLController::vision_ctrl(int Kp, Eigen::Vector3d cPo, Eigen::Vector3d sd)
{
    unsigned int nj = robot_->getNrJnts();
    KDL::JntArray qd(nj);
    qd.data.setZero();

    // 1. Calcolo del vettore s attuale (normalizzato)
    double norm = cPo.norm();
    if (norm < 0.01) return qd; // Sicurezza: se il tag è troppo vicino o nullo, ferma
    Eigen::Vector3d s = cPo / norm;

    // 2. Errore di puntamento: differenza tra direzione attuale e desiderata
    // Usiamo (s - sd) per la legge di controllo
    Eigen::Vector3d error = s - sd;

    // 3. Matrice di Rotazione EE rispetto alla Base
    Eigen::Matrix3d Rc = toEigen(robot_->getEEFrame().M);
    
    // Matrice di rotazione 6x6 per trasformare velocità da Camera a Base
    Eigen::Matrix<double,6,6> R = Eigen::Matrix<double,6,6>::Zero();
    R.block<3, 3>(0, 0) = Rc;
    R.block<3, 3>(3, 3) = Rc;

    // 4. Matrice di Interazione L (Interaction Matrix) definita nel frame camera
    Eigen::Matrix<double,3,3> L1 = -1.0/norm * (Eigen::Matrix3d::Identity() - s*s.transpose());
    Eigen::Matrix3d S_skew;
    S_skew << 0, -s.z(), s.y(), 
              s.z(), 0, -s.x(), 
              -s.y(), s.x(), 0;

    Eigen::Matrix<double,3,6> L;
    L.block<3, 3>(0, 0) = L1;
    L.block<3, 3>(0, 3) = S_skew;

    // 5. Proiezione dello Jacobiano nel frame camera
    // L è in camera frame, J è in base frame. Trasformiamo J: J_cam = R.transpose() * J_base
    Eigen::MatrixXd J = robot_->getEEJacobian().data;
    Eigen::MatrixXd LJ = L * (R.transpose() * J);

    // 6. Null Space per gestione limiti giunti (Manteniamo la tua logica)
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(nj,nj);
    Eigen::MatrixXd JntLimits_ = robot_->getJntLimits();
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd q0_dot(nj);
    double lambda = 50.0;

    for (unsigned int i = 0; i < nj; i++) {
        double range = (JntLimits_(i,1) - JntLimits_(i,0));
        double L_dist = range * range;
        double G = (2*q(i) - JntLimits_(i,1) - JntLimits_(i,0));
        double D = (JntLimits_(i,1) - q(i)) * (q(i) - JntLimits_(i,0));
        q0_dot(i) = (1.0/lambda) * (L_dist * G) / (D * D + 0.001); // 0.001 evita div by zero
    }

    // 7. Legge di controllo finale
    // q_dot = -Kp * pinv(LJ) * error + (I - pinv(J)*J) * q0_dot
    Eigen::MatrixXd LJ_pinv = LJ.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::MatrixXd J_pinv = J.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::MatrixXd N = I - J_pinv * J;

    qd.data = -(double)Kp * LJ_pinv * error + N * q0_dot;

    return qd;
}








/*
KDL::JntArray KDLController::vision_ctrl(int Kp, Eigen::Vector3d cPo, Eigen::Vector3d sd)
{
    unsigned int nj = robot_->getNrJnts();
    KDL::JntArray qd(nj);
    
    // 1. Calcolo del vettore s attuale (normalizzato)
    Eigen::Vector3d s = cPo / cPo.norm();

    // 2. ERRORE: Questa è la chiave. L'errore è la differenza tra s e sd.
    // In Visual Servoing, vogliamo minimizzare l'errore: e = s - sd
    Eigen::Vector3d error = s - sd;

    // 3. Matrice di Rotazione (EE -> Base)
    Eigen::Matrix3d Rc = toEigen(robot_->getEEFrame().M);
    Eigen::Matrix<double,6,6> R = Eigen::Matrix<double,6,6>::Zero();
    R.block<3, 3>(0, 0) = Rc;
    R.block<3, 3>(3, 3) = Rc;

    // 4. Matrice di Interazione L (nel frame camera)
    Eigen::Matrix<double,3,3> L1 = -1.0/cPo.norm() * (Eigen::Matrix3d::Identity() - s*s.transpose());
    Eigen::Matrix3d S_skew;
    S_skew << 0, -s.z(), s.y(), s.z(), 0, -s.x(), -s.y(), s.x(), 0;

    Eigen::Matrix<double,3,6> L;
    L.block<3, 3>(0, 0) = L1;
    L.block<3, 3>(0, 3) = S_skew;

    // 5. Proiezione dello Jacobiano nel frame camera: L * (R_transpose * J)
    // Usiamo R.transpose() perché J è in base frame e vogliamo portarlo in camera/EE frame
    Eigen::MatrixXd J = robot_->getEEJacobian().data;
    Eigen::MatrixXd LJ = L * (R.transpose() * J);

    // 6. Null Space (Gestione Limiti Giunti) - Questo pezzo che avevi è corretto
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(nj,nj);
    Eigen::MatrixXd JntLimits_ = robot_->getJntLimits();
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd q0_dot(nj);
    double lambda = 50.0;

    for (unsigned int i = 0; i < nj; i++) {
        double L_dist = (JntLimits_(i,1) - JntLimits_(i,0)) * (JntLimits_(i,1) - JntLimits_(i,0));
        double G = (2*q(i) - JntLimits_(i,1) - JntLimits_(i,0));
        double D = (JntLimits_(i,1) - q(i)) * (q(i) - JntLimits_(i,0));
        q0_dot(i) = (1.0/lambda) * (L_dist * G) / (D * D);
    }

    // 7. Legge di Controllo Finale
    // q_dot = -Kp * pseudoinverse(L*J) * error + (I - J_inv*J)*q0_dot
    Eigen::MatrixXd LJ_pinv = LJ.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::MatrixXd J_pinv = J.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::MatrixXd N = I - J_pinv * J;

    // NOTA: Il meno davanti a Kp è fondamentale perché vogliamo andare VERSO l'errore zero
    qd.data = -(double)Kp * LJ_pinv * error + N * q0_dot;

    return qd;
}


*/


/*

KDL::JntArray KDLController::vision_ctrl(int Kp, Eigen::Vector3d cPo,Eigen::Vector3d sd )
{
    unsigned int nj = robot_->getNrJnts();

    Eigen::Matrix<double,3,3> Rc;
    Rc = toEigen(robot_->getEEFrame().M);//assumiamo che la matrice di rotazione siano approssimabili
    Eigen::MatrixXd K(nj,nj);
    K = 3*Kp*K.Identity(nj,nj);

    Eigen::Matrix<double,6,6> R =Eigen::Matrix<double,6,6>::Zero();

    R.block<3, 3>(0, 0) = Rc;//.transpose();
    R.block<3, 3>(3, 3) = Rc;//.transpose();

    Eigen::Vector3d s;
    for (int i=0; i<3; i++){
        s(i) = cPo(i)/cPo.norm();
    }
    
    
    RCLCPP_INFO(rclcpp::get_logger("KDLController"),
                "vector s: %f %f %f",
                s(0),s(1),s(2));
    
    Eigen::Matrix<double,3,3> L1;
    L1 = -1/cPo.norm()* (Eigen::Matrix3d::Identity() - s*s.transpose());

    Eigen::Matrix3d S_skew = Eigen::Matrix3d::Zero();
    S_skew <<     0, -s.z(),  s.y(),
                 s.z(),      0, -s.x(),
                -s.y(),  s.x(),      0;


    Eigen::Matrix<double,3,3> L2;
    L2 = S_skew;

    Eigen::Matrix<double,3,6> L;

    L.block<3, 3>(0, 0) = L1;
    L.block<3, 3>(0, 3) = L2;
    
    L = L*R;

    Eigen::MatrixXd J;
    J = robot_->getEEJacobian().data;
    Eigen::MatrixXd Jc; 
    Jc  = J; //assumiamo che i due jacobiani siano uguali

    
    Eigen::MatrixXd I;
    I = Eigen::MatrixXd::Identity(nj,nj);

    Eigen::MatrixXd JntLimits_ (nj,2);
    JntLimits_ = robot_->getJntLimits();

    Eigen::VectorXd q_min(nj);
    Eigen::VectorXd q_max(nj);
    q_min = JntLimits_.col(0);
    q_max = JntLimits_.col(1);

    Eigen::VectorXd q(nj);
    q  = robot_->getJntValues();

    double lambda = 50;

    Eigen::VectorXd q0_dot(nj);
    for (unsigned int i = 0; i<nj; i++) {
        
        double L =(q_max(i) - q_min(i))*(q_max(i) - q_min(i));

        double G = (2*q(i) - q_max(i) - q_min(i));

        double D = (q_max(i)- q(i))*(q(i)- q_min(i));

        q0_dot(i) = 1/lambda*L*G/(D*D);

    }

    Eigen::MatrixXd N (nj,nj);

    N = I - pseudoinverse(J)*J;
    
    Eigen::MatrixXd J_pinv = pseudoinverse(L*J);
    KDL::JntArray qd(nj);
    qd.data =  K*J_pinv*sd + N * q0_dot;

   
    return qd;
}
*/




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

    // 1. Calcolo distanza e vettore direzione
    double distance = cPo.norm();
    if (distance < 0.01) return qd; // Sicurezza per evitare divisioni per zero
    Eigen::Vector3d s = cPo / distance;

    // 2. Errore di puntamento (sd è tipicamente [0,0,1])
    Eigen::Vector3d error = s - sd;

    // 3. Matrici di Rotazione e Jacobiano
    Eigen::Matrix3d Rc = toEigen(robot_->getEEFrame().M);
    Eigen::Matrix<double,6,6> R = Eigen::Matrix<double,6,6>::Zero();
    R.block<3, 3>(0, 0) = Rc;
    R.block<3, 3>(3, 3) = Rc;
    
    Eigen::MatrixXd J = robot_->getEEJacobian().data;
    Eigen::MatrixXd J_pinv = J.completeOrthogonalDecomposition().pseudoInverse();

    // 4. Matrice di Interazione L (Camera Frame)
    Eigen::Matrix<double,3,3> L1 = -1.0/distance * (Eigen::Matrix3d::Identity() - s*s.transpose());
    Eigen::Matrix3d S_skew;
    S_skew << 0, -s.z(), s.y(), 
              s.z(), 0, -s.x(), 
              -s.y(), s.x(), 0;

    Eigen::Matrix<double,3,6> L;
    L.block<3, 3>(0, 0) = L1;
    L.block<3, 3>(0, 3) = S_skew;

    // 5. Proiezione Jacobiano: L_cam * J_cam
    Eigen::MatrixXd LJ = L * (R.transpose() * J);
    Eigen::MatrixXd LJ_pinv = LJ.completeOrthogonalDecomposition().pseudoInverse();

    // --- LOGICA DI AVANZAMENTO DEPOTENZIATA PER STABILITÀ ---
    double v_z = 0.0;
    double target_distance = 0.12; 
    
    if (distance > target_distance) {
        // Usiamo un guadagno molto basso (0.15) per l'avvicinamento
        v_z = 0.15 * (distance - target_distance); 
        
        // LIMITATORE: Non permettiamo al robot di andare più veloce di 4 cm/s
        // Questo impedisce al tag di "schizzare" via dall'inquadratura
        if (v_z > 0.04) v_z = 0.04; 
    }
    
    Eigen::Matrix<double,6,1> v_cam;
    v_cam << 0, 0, v_z, 0, 0, 0; 

    // Trasformazione in Joint Space per l'avanzamento
    Eigen::VectorXd qd_forward = J_pinv * (R * v_cam);

    // 6. Null Space per limiti giunti (logica originale)
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
        q0_dot(i) = (1.0/lambda) * (L_dist * G) / (D * D + 0.001);
    }
    Eigen::MatrixXd N = I - J_pinv * J;

    // 7. Legge di controllo finale
    // Ridotto il peso del Kp sul puntamento per evitare rotazioni brusche
    double Kp_soft = (double)Kp * 0.8; 
    qd.data = -Kp_soft * LJ_pinv * error + qd_forward + N * q0_dot;

    return qd;
}

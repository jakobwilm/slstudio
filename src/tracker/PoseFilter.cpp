#include "PoseFilter.h"

//#include <QCoreApplication>
#include <QTime>
#include <iostream>

//#include <QMetaType>

#include <Eigen/Geometry>

#include <vector>


PoseFilter::PoseFilter(){

    // Setup a 6 dof Kalman filter according to http://campar.in.tum.de/Chair/KalmanFilter
    // The state is 19 dimensional, containing linear position, velocity, accelerations and quaternion, angular velocities and accelerations
    // The measurement dimensionality is 7 containing linear position and quaternion
    // The control matrix is empty, as no control of the system is undertaken
//std::cout << "PoseFilter Constructor..." << std::endl;
    // Set initial state
    s.setZero();
    s(3) = 1.0;

    // Set process variances
    Q = 0.5 * Eigen::Matrix<float, 13, 13>::Identity();

    // Set measurement variances
    R.setZero();
    R = 1.0 * Eigen::Matrix<float, 7, 7>::Identity();

    std::cout << "Q" << std::endl;
    std::cout << Q << std::endl;

    std::cout << "R" << std::endl;
    std::cout << R << std::endl;

    // Initialize error covariance
    P.setZero();
    P.topLeftCorner(7,7) = 1.0*Eigen::Matrix<float, 7, 7>::Identity();
    P.bottomRightCorner(6,6) = 100.0*Eigen::Matrix<float, 6, 6>::Identity();

    std::cout << "P" << std::endl;
    std::cout << P << std::endl;

    time.start();
}

void PoseFilter::filterPoseEstimate(const Eigen::Affine3f &Told, Eigen::Affine3f &Tnew){

    // get measurement
    Eigen::Matrix<float, 7, 1> z;
    z.head(3) = Told.translation();
    Eigen::Quaternionf qtemp(Told.rotation());
    z(3) = qtemp.w();
    z.tail(3) = qtemp.vec();

    // time delta (should really come from camera)
    float tau = (float)time.restart()/1000.0;

    // prediction of state
    s.head(3) = s.head(3) + tau*s.segment(7, 3);

    Eigen::Vector3f omega = s.segment(10, 3);
    float ox = omega(0);
    float oy = omega(1);
    float oz = omega(2);
    float onorm = std::max(omega.norm(), 0.00001f);
//std::cout << "omega: " << std::endl << omega << std::endl;
    Eigen::Matrix4f OmegaBar;
    OmegaBar << 0, -ox, -oy, -oz,
                ox, 0, -oz, oy,
                oy, oz, 0, -ox,
                oz, -oy, ox, 0;
//    OmegaBar << 0, -ox, -oy, -oz,
//                ox, 0, oz, -oy,
//                oy, -oz, 0, ox,
//                oz, oy, -ox, 0;
    OmegaBar = 0.5*OmegaBar;

//std::cout << "OmegaBar: " << std::endl << OmegaBar << std::endl;

    Eigen::Matrix4f Qtran = cos(onorm*tau/2.0)*Eigen::Matrix4f::Identity() + 2.0/onorm*sin(onorm*tau/2.0)*OmegaBar;

//std::cout << "Qtran: " << std::endl << Qtran << std::endl;

    Eigen::Vector4f q = s.segment(3, 4);
    q = Qtran*q;
//    q.normalize();
    s.segment(3, 4) = q;

    // quaternion rotation partial derivatives
    Eigen::Matrix4f dOdx;
    dOdx << 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0;
    Eigen::Matrix4f dOdy;
    dOdy << 0, 0, -1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, -1, 0, 0;
    Eigen::Matrix4f dOdz;
    dOdz << 0, 0, 0, -1, 0, 0, -1, 0, 0, 1, 0, 0, 1, 0, 0, 0;

    Eigen::Matrix<float, 4, 3> Qomega;
    Qomega.col(0) = (-ox*tau/(2*onorm) * sin(onorm*tau/2.0) * Eigen::Matrix4f::Identity() +
                     (ox*tau/(onorm*onorm) * cos(onorm*tau/2.0) - 2.0*ox/(onorm*onorm*onorm) * sin(onorm*tau/2.0)) * OmegaBar +
                     2.0/onorm * sin(onorm*tau/2.0) * dOdx) * q;
    Qomega.col(1) = (-oy*tau/(2*onorm) * sin(onorm*tau/2.0) * Eigen::Matrix4f::Identity() +
                     (oy*tau/(onorm*onorm) * cos(onorm*tau/2.0) - 2.0*oy/(onorm*onorm*onorm) * sin(onorm*tau/2.0)) * OmegaBar +
                     2.0/onorm * sin(onorm*tau/2.0) * dOdy) * q;
    Qomega.col(2) = (-oz*tau/(2*onorm) * sin(onorm*tau/2.0) * Eigen::Matrix4f::Identity() +
                     (oz*tau/(onorm*onorm) * cos(onorm*tau/2.0) - 2.0*oz/(onorm*onorm*onorm) * sin(onorm*tau/2.0)) * OmegaBar +
                     2.0/onorm * sin(onorm*tau/2.0) * dOdz) * q;

//std::cout << "Qomega: " << std::endl << Qomega << std::endl;

    // Jacobian of prediction function
    Eigen::Matrix<float, 13, 13> Phi;
    Phi.setZero();
    Phi.topLeftCorner(6,6) = Eigen::Matrix<float, 6, 6>::Identity();
    Phi.topRightCorner(6,6) = tau*Eigen::Matrix<float, 6, 6>::Identity();
    Phi.block(3,3,4,4) = Qtran;
    Phi.block(3,10,4,3) = Qomega;
    Phi.bottomRightCorner(6,6) = Eigen::Matrix<float, 6, 6>::Identity();

//std::cout << "Phi: " << std::endl << Phi << std::endl;

    // Jacobian of measurement function
    Eigen::Matrix<float, 7, 13> H;
    H.setZero();
    H.topLeftCorner(3,3) = Eigen::Matrix<float, 3, 3>::Identity();
    float qnorm = q.norm();
    H(3,3) = (qnorm*qnorm - q(0))/(qnorm*qnorm*qnorm);
    H(4,4) = (qnorm*qnorm - q(1))/(qnorm*qnorm*qnorm);
    H(5,5) = (qnorm*qnorm - q(2))/(qnorm*qnorm*qnorm);
    H(6,6) = (qnorm*qnorm - q(3))/(qnorm*qnorm*qnorm);

//std::cout << "H: " << std::endl << H << std::endl;
    // prediction of covariance
    P = Phi*P*Phi.transpose() + Q;
//std::cout << "P: " << std::endl << P << std::endl;
    // prediction of measurement
    Eigen::VectorXf h = s.head(7);

    // Kalman gain
    Eigen::MatrixXf K = P*H.transpose()*(H*P*H.transpose() + R).inverse();
//std::cout << "K: " << std::endl << K << std::endl;
    // update state estimate
    s = s + K*(z - h);

    // renormalize quaternion
    q = s.segment(3, 4);
    q.normalize();
    s.segment(3, 4) = q;
//std::cout << "s" << std::endl << s << std::endl;
    // update covariance
    P = (Eigen::Matrix<float, 13, 13>::Identity() - K*H)*P;

    // write updated measurement estimate
    Tnew.translation() = s.head(3);
    qtemp.w() = s(3);
    qtemp.vec() = s.segment(4, 3);
    Tnew.linear() = qtemp.matrix();

}

// Slot allows the object to work in a processing chain (across threads)
void PoseFilter::filterPoseEstimate(const Eigen::Affine3f & T){
    Eigen::Affine3f Tnew;
    this->filterPoseEstimate(T, Tnew);
    emit newFilteredPoseEstimate(Tnew);
}

PoseFilter::~PoseFilter(){

}

#include "eigentools.h"

namespace eigentools{

// Function to fit two sets of corresponding transformation data.
// Algorithm and notation according to Mili Shah, Comparing two sets ofcorresponding six degree of freedom data, CVIU 2011.
// DTU, 2013, Oline V. Olesen, Jakob Wilm
void fitSixDofData(const std::vector<Eigen::Affine3f> X, const std::vector<Eigen::Affine3f> X_mark, Eigen::Affine3f &H){
// NOT DEBUGGED!
    int N = X.size();
    assert(N == X_mark.size());

    // Mean translations
    Eigen::Vector3f t;
    Eigen::Vector3f t_mark;
    for(int i=0; i<N; i++){
        t += 1.0/N * X[i].translation();
        t_mark += 1.0/N * X[i].translation();
    }

    // Data with mean adjusted translations
    Eigen::MatrixXf X_bar(3, 4*N);
    Eigen::MatrixXf X_mark_bar(3, 4*N);
    for(int i=0; i<N; i++){
        X_bar.block(0,i*4,3,3) = X[i].rotation();
        X_bar.block(0,i*4+3,3,1) = X[i].translation() - t;
        X_mark_bar.block(0,i*4,3,3) = X_mark[i].rotation();
        X_mark_bar.block(0,i*4+3,3,1) = X_mark[i].translation() - t_mark;
    }

    // SVD
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(X_bar*X_mark_bar.transpose(), Eigen::ComputeThinU | Eigen::ComputeThinV);

    Eigen::Matrix3f D = Eigen::Matrix3f::Identity();
    if((svd.matrixV()*svd.matrixU().transpose()).determinant() < 0)
        D(3,3) = -1;

    // Best rotation
    Eigen::Matrix3f Omega = svd.matrixV()*D*svd.matrixU().transpose();

    // Best translation
    Eigen::Vector3f tau = t_mark - Omega*t;

    // Complete solution
    H.linear() = Omega;
    H.translation() = tau;

}


}

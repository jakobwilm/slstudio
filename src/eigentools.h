#ifndef EIGENTOOLS_H
#define EIGENTOOLS_H

#include <eigen3/Eigen/Eigen>
#include <vector>

namespace eigentools{

void fitSixDofData(const std::vector<Eigen::Affine3f> X, const std::vector<Eigen::Affine3f> X_mark, Eigen::Affine3f &H);

}

#endif // EIGENTOOLS_H

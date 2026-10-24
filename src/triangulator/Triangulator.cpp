#include "Triangulator.h"
#include <math.h>
#include <stdexcept>
#include <algorithm>


#ifdef WIN32
    #ifndef NAN
        static const unsigned long __nan[2] = {0xffffffff, 0x7fffffff};
    #define NAN (*(const float *) __nan)
    #endif
#endif

namespace {
cv::Vec3f refineDepth(const GeometricCalibrationData &calibration, const cv::Vec3d &ray,
                     const cv::Vec2d &target, bool horizontal, bool vertical, double depth) {
    const cv::Matx33d rotation(calibration.Rp), intrinsic(calibration.Kp);
    const cv::Vec3d translation(calibration.Tp), direction = rotation * ray;
    const auto &k = calibration.kp;
    const auto residual = [&](double candidateDepth, cv::Vec2d &error) {
        const auto p = candidateDepth * direction + translation;
        if (!std::isfinite(candidateDepth) || candidateDepth <= 0 || p[2] <= 0) return false;
        const double x = p[0] / p[2], y = p[1] / p[2], r2 = x * x + y * y;
        const double radial = 1 + r2 * (k[0] + r2 * (k[1] + r2 * k[4]));
        const double xd = x * radial + 2 * k[2] * x * y + k[3] * (r2 + 2 * x * x);
        const double yd = y * radial + k[2] * (r2 + 2 * y * y) + 2 * k[3] * x * y;
        error = {horizontal ? intrinsic(0, 0) * xd + intrinsic(0, 2) - target[0] : 0,
                 vertical ? intrinsic(1, 1) * yd + intrinsic(1, 2) - target[1] : 0};
        return std::isfinite(error[0]) && std::isfinite(error[1]);
    };
    bool converged = false;
    for (int iteration = 0; iteration < 20; ++iteration) {
        cv::Vec2d error, before, after;
        if (!residual(depth, error)) break;
        const double cost = error.dot(error);
        if (cost < 1e-8) { converged = true; break; }
        const double delta = depth * 1e-4;
        if (!residual(depth - delta, before) || !residual(depth + delta, after)) break;
        const cv::Vec2d derivative = (after - before) / (2 * delta);
        const double denominator = derivative.dot(derivative);
        if (!std::isfinite(denominator) || denominator < 1e-24) break;
        double step = error.dot(derivative) / denominator;
        if (std::abs(step) <= depth * 1e-8) { converged = true; break; }
        step = std::clamp(step, -depth * 0.5, depth * 0.5);
        bool improved = false;
        for (int trial = 0; trial < 12; ++trial, step *= 0.5) {
            cv::Vec2d candidate;
            if (residual(depth - step, candidate) && candidate.dot(candidate) < cost) {
                depth -= step;
                improved = true;
                break;
            }
        }
        if (!improved) break;
    }
    return converged ? cv::Vec3f(depth * ray) : cv::Vec3f(NAN, NAN, NAN);
}
}

TriangulatorOrganized::TriangulatorOrganized(const GeometricCalibrationData &requestedSettings, bool correction)
    : calibration(requestedSettings), correctProjectorDistortion(correction){

    if (calibration.frameWidth < 1 || calibration.frameHeight < 1)
        throw std::invalid_argument("Calibration camera dimensions must be positive");
    if (correction && (!cv::checkRange(cv::Mat(calibration.Kc)) ||
        calibration.Kc(0, 0) <= 0 || calibration.Kc(1, 1) <= 0 || !cv::checkRange(cv::Mat(calibration.Kp)) ||
        !cv::checkRange(cv::Mat(calibration.kp)) || !cv::checkRange(cv::Mat(calibration.Rp)) ||
        !cv::checkRange(cv::Mat(calibration.Tp)) || calibration.Kp(0, 0) <= 0 || calibration.Kp(1, 1) <= 0))
        throw std::invalid_argument("Projector distortion correction requires finite calibration and positive focal lengths");

    // Precompute uc, vc maps
    uc.create(calibration.frameHeight, calibration.frameWidth, CV_32F);
    vc.create(calibration.frameHeight, calibration.frameWidth, CV_32F);

    for(int row=0; row<calibration.frameHeight; row++){
        for(int col=0; col<calibration.frameWidth; col++){
            uc.at<float>(row, col) = col;
            vc.at<float>(row, col) = row;
        }
    }

    // Precompute determinant tensor
    cv::Mat Pc(3,4,CV_32F,cv::Scalar(0.0));
    cv::Mat(calibration.Kc).copyTo(Pc(cv::Range(0,3), cv::Range(0,3)));

    cv::Mat Pp(3,4,CV_32F), temp(3,4,CV_32F);
    cv::Mat(calibration.Rp).copyTo(temp(cv::Range(0,3), cv::Range(0,3)));
    cv::Mat(calibration.Tp).copyTo(temp(cv::Range(0,3), cv::Range(3,4)));
    Pp = cv::Mat(calibration.Kp) * temp;

    cv::Mat e = cv::Mat::eye(4, 4, CV_32F);

    int sz[] = {4, 3, 3, 3};
    cv::Mat C(4, sz, CV_32F, cv::Scalar::all(0));
    for(int k=0; k<4; k++){
        for(int i=0; i<3; i++){
            for(int j=0; j<3; j++){
                for(int l=0; l<3; l++){
                    cv::Mat op(4, 4, CV_32F);
                    Pc.row(i).copyTo(op.row(0));
                    Pc.row(j).copyTo(op.row(1));
                    Pp.row(l).copyTo(op.row(2));
                    e.row(k).copyTo(op.row(3));
                    C.at<float>(cv::Vec4i(k,i,j,l)) = cv::determinant(op.t());
                }
            }
        }
    }
    determinantTensor = C;

    // Precompute lens correction maps
    cv::Mat eye = cv::Mat::eye(3, 3, CV_32F);
    cv::initUndistortRectifyMap(calibration.Kc, calibration.kc, eye, calibration.Kc, cv::Size(calibration.frameWidth, calibration.frameHeight),  CV_16SC2, lensMap1, lensMap2);

    //cv::Mat map1, map2;
    //cv::normalize(lensMap1, map1, 0, 255, cv::NORM_MINMAX, CV_8U);
    //cv::normalize(lensMap2, map2, 0, 255, cv::NORM_MINMAX, CV_8U);
    //cv::imwrite("map1.png", map1);
    //cv::imwrite("map2.png", map2);

    // Precompute parts of xyzw
    xyzwPrecomputeOffset.resize(4);
    xyzwPrecomputeFactor.resize(4);
    for(unsigned int i=0; i<4; i++){
        xyzwPrecomputeOffset[i] = C.at<float>(cv::Vec4i(i,0,1,0)) - C.at<float>(cv::Vec4i(i,2,1,0))*uc - C.at<float>(cv::Vec4i(i,0,2,0))*vc;
        xyzwPrecomputeFactor[i] = - C.at<float>(cv::Vec4i(i,0,1,2)) + C.at<float>(cv::Vec4i(i,2,1,2))*uc + C.at<float>(cv::Vec4i(i,0,2,2))*vc;
    }
}

TriangulationResult TriangulatorOrganized::triangulate(const cv::Mat1f &up, const cv::Mat1f &vp,
                                             const cv::Mat1b &mask, const cv::Mat1b &shading) const {
    cv::Mat horizontal = up, vertical = vp, valid = mask, correctedShading = shading, xyz;
    triangulate(horizontal, vertical, valid, correctedShading, xyz);
    return {cv::Mat3f(xyz), cv::Mat1b(correctedShading)};
}

void TriangulatorOrganized::triangulate(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading, cv::Mat &pointCloud) const {

    const cv::Size expected(calibration.frameWidth, calibration.frameHeight);
    if ((up.empty() && vp.empty()) ||
        (!up.empty() && (up.dims != 2 || up.type() != CV_32FC1 || up.size() != expected)) ||
        (!vp.empty() && (vp.dims != 2 || vp.type() != CV_32FC1 || vp.size() != expected)) ||
        mask.dims != 2 || mask.type() != CV_8UC1 || mask.size() != expected ||
        shading.dims != 2 || shading.type() != CV_8UC1 || shading.size() != expected)
        throw std::invalid_argument("Triangulation inputs do not match calibration");

    // Undistort up, mask and shading
    if(!up.empty()){
        cv::Mat upUndistort;
        cv::remap(up, upUndistort, lensMap1, lensMap2, cv::INTER_LINEAR);
        up = upUndistort;
    }
    if(!vp.empty()){
        cv::Mat vpUndistort;
        cv::remap(vp, vpUndistort, lensMap1, lensMap2, cv::INTER_LINEAR);
        vp = vpUndistort;
    }

    cv::Mat maskUndistort, shadingUndistort;
    cv::remap(mask, maskUndistort, lensMap1, lensMap2, cv::INTER_NEAREST);
    cv::remap(shading, shadingUndistort, lensMap1, lensMap2, cv::INTER_LINEAR);
    mask = maskUndistort;
    shading = shadingUndistort;

    // Triangulate
    cv::Mat xyz;
    if(!up.empty() && vp.empty())
        triangulateFromUp(up, xyz);
    else if(!vp.empty() && up.empty())
        triangulateFromVp(vp, xyz);
    else if(!up.empty() && !vp.empty())
        triangulateFromUpVp(up, vp, mask, xyz);

    if (correctProjectorDistortion && cv::norm(calibration.kp) > 0)
        refineProjectorDistortion(up, vp, mask, xyz);

    // Mask
    xyz.setTo(cv::Scalar(NAN, NAN, NAN), mask == 0);
    pointCloud = xyz;

}

void TriangulatorOrganized::refineProjectorDistortion(const cv::Mat &up, const cv::Mat &vp,
                                            const cv::Mat &mask, cv::Mat &xyz) const {
    for (int row = 0; row < xyz.rows; ++row) {
        for (int col = 0; col < xyz.cols; ++col) {
            if (!mask.at<unsigned char>(row, col)) continue;
            auto &point = xyz.at<cv::Vec3f>(row, col);
            const cv::Vec3d ray((col - calibration.Kc(0, 2)) / calibration.Kc(0, 0),
                               (row - calibration.Kc(1, 2)) / calibration.Kc(1, 1), 1);
            const cv::Vec2d target(up.empty() ? 0 : up.at<float>(row, col),
                                   vp.empty() ? 0 : vp.at<float>(row, col));
            point = refineDepth(calibration, ray, target, !up.empty(), !vp.empty(), point[2]);
        }
    }
}

TriangulatorUnorganized::TriangulatorUnorganized(const GeometricCalibrationData &requestedSettings, bool correction)
    : calibration(requestedSettings), correctProjectorDistortion(correction) {
    if (calibration.frameWidth < 1 || calibration.frameHeight < 1 ||
        calibration.screenResX < 1 || calibration.screenResY < 1 ||
        !cv::checkRange(cv::Mat(calibration.Kc)) || !cv::checkRange(cv::Mat(calibration.kc)) ||
        !cv::checkRange(cv::Mat(calibration.Kp)) || !cv::checkRange(cv::Mat(calibration.Rp)) ||
        !cv::checkRange(cv::Mat(calibration.Tp)) ||
        calibration.Kc(0, 0) <= 0 || calibration.Kc(1, 1) <= 0 ||
        calibration.Kp(0, 0) <= 0 || calibration.Kp(1, 1) <= 0 ||
        (correction && !cv::checkRange(cv::Mat(calibration.kp))))
        throw std::invalid_argument("Unorganized triangulation requires finite camera/projector calibration and positive dimensions");
}

PointCloudPtr TriangulatorUnorganized::triangulate(const std::vector<ProjectorTransition> &transitions,
                                                  const PointCloudSettings &settings) const {
    if (!std::isfinite(settings.minDepth) || !std::isfinite(settings.maxDepth) ||
        settings.minDepth > settings.maxDepth)
        throw std::invalid_argument("Invalid point cloud depth range");
    auto cloud = std::make_shared<PointCloud>();
    if (transitions.empty()) return cloud;
    std::vector<cv::Point2d> camera;
    camera.reserve(transitions.size());
    for (const auto &sample : transitions) {
        const int length = sample.direction == CodecDirHorizontal ? calibration.screenResX : calibration.screenResY;
        if ((sample.direction != CodecDirHorizontal && sample.direction != CodecDirVertical) ||
            !std::isfinite(sample.camera.x) || !std::isfinite(sample.camera.y) ||
            sample.camera.x < 0 || sample.camera.y < 0 ||
            sample.camera.x > calibration.frameWidth - 1 || sample.camera.y > calibration.frameHeight - 1 ||
            !std::isfinite(sample.coordinate) || sample.coordinate < -0.5f || sample.coordinate > length - 0.5f)
            throw std::invalid_argument("Invalid subpixel camera/projector transition");
        camera.emplace_back(sample.camera);
    }
    std::vector<cv::Point2d> normalized;
    cv::undistortPoints(camera, normalized, calibration.Kc, calibration.kc, cv::noArray(), cv::noArray(),
                        cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 1e-10));
    const cv::Matx33d rotation(calibration.Rp), intrinsic(calibration.Kp);
    const cv::Vec3d translation(calibration.Tp);
    cloud->points.reserve(transitions.size());
    for (std::size_t index = 0; index < transitions.size(); ++index) {
        const auto &sample = transitions[index];
        const cv::Vec3d ray(normalized[index].x, normalized[index].y, 1);
        const auto direction = rotation * ray;
        const int axis = sample.direction == CodecDirHorizontal ? 0 : 1;
        const cv::Vec3d plane(intrinsic(axis, 0) - sample.coordinate * intrinsic(2, 0),
                             intrinsic(axis, 1) - sample.coordinate * intrinsic(2, 1),
                             intrinsic(axis, 2) - sample.coordinate * intrinsic(2, 2));
        const double denominator = plane.dot(direction);
        if (std::abs(denominator) <= 1e-12 * cv::norm(plane) * cv::norm(direction)) continue;
        const double depth = -plane.dot(translation) / denominator;
        if (!std::isfinite(depth) || depth <= 0 || (depth * direction + translation)[2] <= 0) continue;
        cv::Vec3f point(depth * ray);
        if (correctProjectorDistortion && cv::norm(calibration.kp) > 0) {
            const cv::Vec2d target(axis == 0 ? sample.coordinate : 0, axis == 1 ? sample.coordinate : 0);
            point = refineDepth(calibration, ray, target, axis == 0, axis == 1, depth);
        }
        if (!std::isfinite(point[0]) || !std::isfinite(point[1]) || !std::isfinite(point[2]) ||
            point[2] < settings.minDepth || point[2] > settings.maxDepth) continue;
        cloud->points.push_back({point[0], point[1], point[2], sample.shading, sample.shading, sample.shading});
    }
    return cloud;
}

void TriangulatorOrganized::triangulateFromUp(const cv::Mat &up, cv::Mat &xyz) const {

    // Solve for xyzw using determinant tensor
    cv::Mat C = determinantTensor;
    std::vector<cv::Mat> xyzw(4);
    for(unsigned int i=0; i<4; i++){
//        xyzw[i].create(up.size(), CV_32F);
        xyzw[i] = xyzwPrecomputeOffset[i] + xyzwPrecomputeFactor[i].mul(up);
    }

    // Convert to non homogenous coordinates
    cv::Mat winv;
    cv::divide(1.0, xyzw[3], winv);
    for(unsigned int i=0; i<3; i++)
        xyzw[i] = xyzw[i].mul(winv);

    // Merge
    cv::merge(std::vector<cv::Mat>(xyzw.begin(), xyzw.begin()+3), xyz);

}

void TriangulatorOrganized::triangulateFromVp(const cv::Mat &vp, cv::Mat &xyz) const {

    // Solve for xyzw using determinant tensor
    cv::Mat C = determinantTensor;
    std::vector<cv::Mat> xyzw(4);
    for(unsigned int i=0; i<4; i++){
//        xyzw[i].create(vp.size(), CV_32F);
        xyzw[i] = C.at<float>(cv::Vec4i(i,0,1,1)) - C.at<float>(cv::Vec4i(i,2,1,1))*uc - C.at<float>(cv::Vec4i(i,0,2,1))*vc -
                C.at<float>(cv::Vec4i(i,0,1,2))*vp + C.at<float>(cv::Vec4i(i,2,1,2))*vp.mul(uc) + C.at<float>(cv::Vec4i(i,0,2,2))*vp.mul(vc);
    }

    // Convert to non homogenous coordinates
    cv::Mat winv;
    cv::divide(1.0, xyzw[3], winv);
    for(unsigned int i=0; i<3; i++)
        xyzw[i] = xyzw[i].mul(winv);

    // Merge
    cv::merge(std::vector<cv::Mat>(xyzw.begin(), xyzw.begin()+3), xyz);

}

void TriangulatorOrganized::triangulateFromUpVp(const cv::Mat &up, const cv::Mat &vp,
                                               const cv::Mat &mask, cv::Mat &xyz) const {
    xyz = cv::Mat(up.size(), CV_32FC3, cv::Scalar(NAN, NAN, NAN));
    if (cv::norm(calibration.Tp) == 0) return;

    std::vector<cv::Point2d> camera, projector;
    std::vector<cv::Point> pixels;
    for (int row = 0; row < up.rows; ++row) {
        for (int col = 0; col < up.cols; ++col) {
            const double u = up.at<float>(row, col), v = vp.at<float>(row, col);
            if (!mask.at<unsigned char>(row, col) || !std::isfinite(u) || !std::isfinite(v)) continue;
            camera.emplace_back(col, row);
            projector.emplace_back(u, v);
            pixels.emplace_back(col, row);
        }
    }
    if (pixels.empty()) return;

    if (correctProjectorDistortion && cv::norm(calibration.kp) > 0) {
        std::vector<cv::Point2d> undistorted;
        cv::undistortPoints(projector, undistorted, calibration.Kp, calibration.kp,
                            cv::noArray(), calibration.Kp,
                            cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 1e-10));
        projector = std::move(undistorted);
    }

    const cv::Matx33d cameraIntrinsic(calibration.Kc), projectorIntrinsic(calibration.Kp);
    const cv::Matx33d cameraInverse = cameraIntrinsic.inv(), projectorInverse = projectorIntrinsic.inv();
    const cv::Matx33d rotation(calibration.Rp);
    const cv::Vec3d translation(calibration.Tp);
    std::size_t count = 0;
    for (std::size_t index = 0; index < pixels.size(); ++index) {
        if (!std::isfinite(projector[index].x) || !std::isfinite(projector[index].y)) continue;
        const auto cameraRay = cameraInverse * cv::Vec3d(camera[index].x, camera[index].y, 1);
        const auto projectorRay = rotation.t() * (projectorInverse *
            cv::Vec3d(projector[index].x, projector[index].y, 1));
        // Parallel viewing rays cannot determine a finite depth.
        const double cross = cv::norm(cameraRay.cross(projectorRay));
        if (!std::isfinite(cross) || cross <= 1e-12 * cv::norm(cameraRay) * cv::norm(projectorRay)) continue;
        camera[count] = camera[index];
        projector[count] = projector[index];
        pixels[count++] = pixels[index];
    }
    camera.resize(count);
    projector.resize(count);
    pixels.resize(count);
    if (pixels.empty()) return;

    const cv::Matx34d cameraExtrinsic(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0);
    const cv::Matx34d projectorExtrinsic(
        rotation(0, 0), rotation(0, 1), rotation(0, 2), translation[0],
        rotation(1, 0), rotation(1, 1), rotation(1, 2), translation[1],
        rotation(2, 0), rotation(2, 1), rotation(2, 2), translation[2]);
    cv::Mat homogeneous;
    cv::triangulatePoints(cameraIntrinsic * cameraExtrinsic, projectorIntrinsic * projectorExtrinsic,
                          camera, projector, homogeneous);

    for (std::size_t index = 0; index < pixels.size(); ++index) {
        const int column = static_cast<int>(index);
        const double w = homogeneous.at<double>(3, column);
        if (!std::isfinite(w) || w == 0) continue;
        const cv::Vec3d point(homogeneous.at<double>(0, column) / w,
                              homogeneous.at<double>(1, column) / w,
                              homogeneous.at<double>(2, column) / w);
        if (!std::isfinite(point[0]) || !std::isfinite(point[1]) || !std::isfinite(point[2]) ||
            point[2] <= 0 || (rotation * point + translation)[2] <= 0) continue;
        const cv::Vec3f output(point);
        if (std::isfinite(output[0]) && std::isfinite(output[1]) && std::isfinite(output[2]))
            xyz.at<cv::Vec3f>(pixels[index]) = output;
    }
}

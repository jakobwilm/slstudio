#include "CalibrationWorker.h"
#include "CodecPhaseShift2x3.h"

#include "cvtools.h"

#include "CodecCalibration.h"
#include "RBFInterpolator.h"
#include <QDateTime>
#include <QSettings>

#define sc static_cast
#define dc dynamic_cast

#define DEBUG_OUTPUT false

CalibrationWorker::CalibrationWorker(QObject *parent, unsigned int _screenCols,
                                     unsigned int _screenRows)
    : QObject(parent), screenCols(_screenCols), screenRows(_screenRows) {

  decoder.reset(new DecoderCalibration(screenCols, screenRows, CodecDirBoth));

  cv::SimpleBlobDetector::Params parameters;
  parameters.blobColor = 255;
  //  parameters.filterByArea = true;
  //  parameters.filterByColor = true;
  //  parameters.filterByCircularity = true;
  //  parameters.filterByInertia = true;
  //  parameters.filterByConvexity = true;
  //  parameters.minArea = 50;
  //  parameters.maxArea = 2000;
  //  parameters.minCircularity = 0.85f; // note: very sensitive parameter.
  //  parameters.maxCircularity = 1.1f;
  //  parameters.minInertiaRatio = 0.4f;
  //  parameters.maxInertiaRatio = 1.1f;
  //  parameters.minConvexity = 0.9f;
  //  parameters.maxConvexity = 1.1f;

  blobDetector = cv::SimpleBlobDetector::create(parameters);
}

static std::vector<size_t>
findNearestNeighborsAngleSorted(const cv::KeyPoint queryPoint,
                                const std::vector<cv::KeyPoint> searchPoints,
                                const int N) {

  struct neighbour {
    size_t idx;
    float distSq;
    float angle;
  };

  // Brute force NN search
  std::vector<neighbour> nn;
  for (auto j = 0u; j < searchPoints.size(); j++) {
    if (queryPoint.pt == searchPoints[j].pt)
      continue;

    float distSq =
        sc<float>(cv::norm(cv::Vec2f(queryPoint.pt),
                           cv::Vec2f(searchPoints[j].pt), cv::NORM_L2SQR));

    neighbour n{j, distSq, 0};

    if (nn.size() < sc<size_t>(N)) {
      nn.push_back(n);
    } else {
      if (distSq < nn[sc<size_t>(N - 1)].distSq) {
        nn[sc<size_t>(N - 1)] = n;
      }
    }

    std::sort(nn.begin(), nn.end(),
              [](neighbour a, neighbour b) { return a.distSq < b.distSq; });
  }

  // Now sort neighbours according to angle
  for (auto &n : nn) {
    n.angle = std::atan2(searchPoints[n.idx].pt.x - queryPoint.pt.x,
                         searchPoints[n.idx].pt.y - queryPoint.pt.y);
  }
  std::sort(nn.begin(), nn.end(),
            [](neighbour a, neighbour b) { return a.angle < b.angle; });

  // Return idx vector
  std::vector<size_t> idxs;
  for (auto n : nn)
    idxs.push_back(n.idx);

  return idxs;
}

static cv::Mat fitHomography(std::vector<cv::Point2f> q1,
                             std::vector<cv::Point2f> q2, float &ssd) {

  cv::Mat H = cv::findHomography(q1, q2, 0);

  std::vector<cv::Point2f> q1T = cvtools::applyHomTransform(H, q1);

  ssd = 0;
  for (auto i = 0u; i < q1.size(); i++) {
    ssd +=
        static_cast<float>(cv::norm(cv::Vec2f(q1T[i] - q2[i]), cv::NORM_L2SQR));
  }

  return H;
}

bool CalibrationWorker::findPartialCirclesGrid(const cv::Mat &im,
                                               std::vector<cv::Point2f> &q,
                                               std::vector<cv::Point3f> &Q,
                                               const float circleSpacing) {

  std::vector<cv::KeyPoint> keypoints;
  blobDetector->detect(im, keypoints);

  cv::imwrite("im.png", im);

  if (keypoints.size() < 5)
    return false;

  // remove keypoint too close to image border
  std::vector<cv::KeyPoint> keypointsFiltered;
  for (auto &k : keypoints) {
    const float d = 3; // px
    float r = k.size / 2.0f;
    if (k.pt.x > (r + d) && k.pt.x < (im.cols - (r + d)) && k.pt.y > (r + d) &&
        k.pt.y < (im.rows - (r + d)))
      keypointsFiltered.emplace_back(k);
  }
  keypoints = keypointsFiltered;

  // Move through all detections
  std::vector<std::pair<cv::Point, cv::Point2f>> corrBest;
  for (auto i = 0u; i < keypoints.size(); i++) {

    std::vector<size_t> nn =
        findNearestNeighborsAngleSorted(keypoints[i], keypoints, 4);

    // center, neighborhood
    std::vector<cv::Point2f> qIm{keypoints[i].pt};
    for (auto &n : nn)
      qIm.push_back(keypoints[n].pt);

    // Assuming the nine points form a "8-neighborhood", we fit a homography
    //    std::vector<cv::Point2f> qObj = {{0, 0}, {-1, 0}, {-1, 1}, {0, 1}, {1,
    //    1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}};
    std::vector<cv::Point2f> qObj = {{0, 0}, {-1, 0}, {0, 1}, {1, 0}, {0, -1}};

    float ssd;
    cv::Mat H = fitHomography(qIm, qObj, ssd);

    // If ssd is high, we probably did not get the desired 8-neighborhood
    if (ssd > 0.01f)
      continue;

    // Transform image corners
    std::vector<cv::Point2f> corners{{-0.5f, -0.5f},
                                     {im.cols - 0.5f, 0.5f},
                                     {im.cols - 0.5f, im.rows - 0.5f},
                                     {0.5f, im.rows - 0.5f}};

    std::vector<cv::Point2f> cornersObj =
        cvtools::applyHomTransform(cv::Matx33f(H), corners);

    // Get axis-aligned bounding rect
    cv::Rect r = cv::boundingRect(cornersObj);
    int minX = r.x;
    int minY = r.y;
    int maxX = r.x + r.width;
    int maxY = r.y + r.height;

#if DEBUG_OUTPUT
    cv::Mat imColor;
    cv::cvtColor(im, imColor, cv::COLOR_GRAY2BGR);
    cv::drawKeypoints(im, keypoints, imColor);
#endif

    cv::Mat Hinv = H.inv();

    std::vector<std::pair<cv::Point, cv::Point2f>> idealPoints;
    for (int i = minY; i < maxY; i++) {
      for (int j = minX; j < maxX; j++) {

        cv::Point2f imPoint =
            cvtools::applyHomTransform(cv::Matx33f(H).inv(), cv::Point2f(j, i));
        idealPoints.push_back(std::make_pair(cv::Point(j, i), imPoint));
#if DEBUG_OUTPUT
        cv::drawMarker(imColor, imPoint, cv::Scalar(255, 0, 0),
                       cv::MARKER_CROSS, 20, 1);
#endif
      }
    }

#if DEBUG_OUTPUT
    cv::drawMarker(imColor, qIm[0], cv::Scalar(0, 255, 0));
    for (auto i = 1u; i < qIm.size(); i++)
      cv::drawMarker(imColor, qIm[i], cv::Scalar(0, 0, 255));

    cv::imwrite("imColor.png", imColor);
#endif

    // Get correspondences
    std::vector<std::pair<cv::Point, cv::Point2f>> corr;
    for (auto ip : idealPoints) {
      for (auto k : keypoints) {
        if (cv::norm(cv::Vec2f(ip.second - k.pt), cv::NORM_L2SQR) <
            55) { // pxsq

          corr.push_back(std::make_pair(ip.first, k.pt));
        }
      }
    }

    if (corr.size() > corrBest.size()) {
      corrBest = corr;
      //      iBest = i;
    }
  }

  // Fit homography with all current correspondences
  std::vector<cv::Point> idealPoints(corrBest.size());
  q.resize(corrBest.size());

  for (auto i = 0u; i < corrBest.size(); i++) {
    idealPoints[i] = corrBest[i].first;
    q[i] = corrBest[i].second;
  }

  // Offset Q 'indices' such that all are positive
  cv::Rect r = cv::boundingRect(idealPoints);
  Q.resize(corrBest.size());
  for (auto i = 0u; i < corrBest.size(); i++)
    Q[i] = circleSpacing *
           cv::Point3f(idealPoints[i].x - r.x, idealPoints[i].y - r.y, 0.0);

  if (corrBest.size() > 10)
    return true;
  else
    return false;
}

void plotResiduals(const std::vector<std::vector<cv::Point3f>> Q,
                   const std::vector<std::vector<cv::Point2f>> q,
                   const cv::Matx33f K, const cv::Vec<float, 5> k,
                   const std::vector<cv::Mat> rvecs,
                   const std::vector<cv::Mat> tvecs) {

  //  // per-view reprojection errors:
  //  std::vector<float> pve(Q.size());
  //  pve.resize(Q.size());

  // reprojection errors:
  std::vector<cv::Point2d> res;

  for (size_t i = 0; i < Q.size(); ++i) {

    std::vector<cv::Point2f> qProj;
    cv::projectPoints(cv::Mat(Q[i]), rvecs[i], tvecs[i], K, k, qProj);
    //    float err = 0;
    for (unsigned int j = 0; j < qProj.size(); j++) {
      cv::Point2f d = q[i][j] - qProj[j];
      res.push_back(d);
      //      err += cv::sqrt(d.x * d.x + d.y * d.y);
    }
  }

  double minX = -10.0;
  double maxX = 10.0;
  double minY = -10.0;

  cv::Mat resImage =
      cvtools::plotScatterXY(res, cv::Size(1024, 1024), minX, maxX, minY);

  cv::imwrite("resImage.png", resImage);
}

bool CalibrationWorker::calibrate(CalibrationData &cal,
                                  std::vector<std::vector<cv::Mat>> &frameSeqs,
                                  std::vector<size_t> &activeSeqs) {

  QSettings settings;

  // Calibration board parameters
  float spacing = settings.value("calibration/spacing", 0.004).toFloat(); // mm
  //  float circleDiameter =
  //      settings.value("calibration/circleDiameter", 0.002).toFloat();
  auto rows = settings.value("calibration/rows", 10).toInt();
  auto cols = settings.value("calibration/cols", 15).toInt();

  // Number of features on calibration pattern
  cv::Size patternSize(cols, rows);

  // Number of calibration sequences
  auto nFrameSeq = activeSeqs.size();

  std::vector<cv::Mat> up(nFrameSeq), vp(nFrameSeq), shading(nFrameSeq),
      mask(nFrameSeq);

  // Decode frame sequences
  for (unsigned int i = 0; i < nFrameSeq; i++) {
    const std::vector<cv::Mat> &frameSeq = frameSeqs[activeSeqs[i]];

    for (size_t i = 0; i < frameSeq.size(); ++i) {
      decoder->setFrame(i, frameSeq[i]);
    }

    decoder->decodeFrames(up[i], vp[i], mask[i], shading[i]);
#if 0
            cvtools::writeMat(shading[i], QString("shading[%1].mat").arg(i).toLocal8Bit());
            cvtools::writeMat(up[i], QString("up[%1].mat").arg(i).toLocal8Bit());
            cvtools::writeMat(vp[i], QString("vp[%1].mat").arg(i).toLocal8Bit());
#endif
  }

  auto frameWidth = frameSeqs[0][0].cols;
  auto frameHeight = frameSeqs[0][0].rows;

  // Find calibration point coordinates for camera and projector
  std::vector<std::vector<cv::Point2f>> qc, qp;
  std::vector<std::vector<cv::Point3f>> Q;

  std::vector<cv::Mat> results(shading.size());

  for (unsigned int i = 0; i < nFrameSeq; i++) {
    std::vector<cv::Point2f> qci;
    std::vector<cv::Point3f> Qi;

    bool success = false;
    QString pattern =
        settings.value("calibration/pattern", "circles").toString();
    if (pattern == "circles") {
      success = findPartialCirclesGrid(shading[i], qci, Qi, spacing);

      // Draw detections
      cv::cvtColor(shading[i], results[i], cv::COLOR_GRAY2RGB);

      if (success) {

        for (auto j = 0u; j < qci.size(); ++j) {
          cv::drawMarker(results[i], qci[j], cv::Scalar(0, 255, 0));
          //        cv::putText(shadingColor,
          //                    QString("(%1,%2)")
          //                        .arg(sc<double>(Qi[i].x))
          //                        .arg(sc<double>(Qi[i].y))
          //                        .toStdString(),
          //                    qci[i], cv::FONT_HERSHEY_PLAIN, 1.0,
          //                    cv::Scalar(255, 0, 0));
        }

      } else {

        std::cout << "Could not find regular circles grid " << std::endl
                  << std::flush;

        std::vector<cv::KeyPoint> keypoints;
        blobDetector->detect(shading[i], keypoints);
        cv::imwrite("shadingi.png", shading[i]);
        cv::drawKeypoints(shading[i], keypoints, results[i]);
      }
    } else if (pattern == "checkers") {
      success =
          cv::findChessboardCorners(shading[i], cv::Size(rows, cols), qci);
      for (int h = 0; h < patternSize.height; h++) {
        for (int w = 0; w < patternSize.width; w++) {
          Qi.push_back(cv::Point3f(spacing * w, spacing * h, 0.0));
        }
      }

      cv::drawChessboardCorners(shading[i], cv::Size(rows, cols), qci, success);
    }
#ifdef _DEBUG
    cv::imwrite("shadingColor.png", results[i]);
#endif

    // Emit chessboard results
    emit newSequenceResult(results[i], activeSeqs[i], success);

    if (success) {
      // Vectors of accepted points for current view
      std::vector<cv::Point2f> qpi_a;
      std::vector<cv::Point2f> qci_a;
      std::vector<cv::Point3f> Qi_a;

      // Loop through features
      size_t j = 0;
      for (const auto &qcij : qci) {

        // Collect neighbor points
        const int SEARCH_RADIUS = 10;
        const int WINDOW_SIZE = SEARCH_RADIUS;
        std::vector<cv::Point2f> N_qcij, N_qpij;

        cv::Rect imRect(0, 0, frameWidth, frameHeight);
        cv::Rect roi(
            cv::Point(qcij.x + 0.5 - WINDOW_SIZE, qcij.y + 0.5 - WINDOW_SIZE),
            cv::Point(qcij.x + 0.5 + WINDOW_SIZE, qcij.y + 0.5 + WINDOW_SIZE));
        roi = roi & imRect; // intersection of rects

        for (int h = roi.y; h <= roi.y + roi.height; h++) {
          for (int w = roi.x; w <= roi.x + roi.width; w++) {
            // stay within circle and mask
            float centerDistSq =
                (h - qcij.y) * (h - qcij.y) + (w - qcij.x) * (w - qcij.x);
            if ((centerDistSq < SEARCH_RADIUS * SEARCH_RADIUS) &&
                mask[i].at<bool>(h, w)) {

              N_qcij.push_back(cv::Point2f(w, h));

              float upijwh = up[i].at<float>(h, w);
              float vpijwh = vp[i].at<float>(h, w);
              N_qpij.push_back(cv::Point2f(upijwh, vpijwh));
            }
          }
        }

        // if enough valid points to build homography
        if (N_qpij.size() >= 10) {

          QString method =
              settings.value("calibration/method", "homographies").toString();

          if (method == "homographies") {
            // translate qcij into qpij using local homography
            std::vector<uchar> mask_ij;
            cv::Mat H =
                cv::findHomography(N_qcij, N_qpij, cv::LMEDS, 0.5, mask_ij);

            // if not enough correspondences agreed and homography is valid
            if (std::count(mask_ij.begin(), mask_ij.end(), 1) >
                    0.90 * mask_ij.size() &&
                !H.empty()) {

              cv::Point3d Qpij = cv::Point3d(
                  cv::Mat(H * cv::Mat(cv::Point3d(qcij.x, qcij.y, 1.0))));
              cv::Point2f qpij = cv::Point2f(Qpij.x / Qpij.z, Qpij.y / Qpij.z);

              qpi_a.push_back(qpij);
              qci_a.push_back(qcij);
              Qi_a.push_back(Qi[j]);
            }
          } else if (method == "rbf") {

            RBFInterpolator interp(RBF_GAUSSIAN, 100.0);

            // translate qcij into qpij using interpolator
            interp.setDataPoints(N_qcij, N_qpij);
            cv::Point2f qpij = interp.interpolate(N_qcij, qcij);

            qpi_a.push_back(qpij);
            qci_a.push_back(qci[j]);
            Qi_a.push_back(Qi[j]);
          }

          ++j;
        }
      }

      if (!Qi_a.empty()) {
        // Store projector corner coordinates
        qp.push_back(qpi_a);

        // Store camera corner coordinates
        qc.push_back(qci_a);

        // Store world corner coordinates
        Q.push_back(Qi_a);
      }
    }
  }

  if (Q.size() < 1) {
    std::cerr << "Error: not enough calibration sequences!" << std::endl;
    return false;
  }

  // calibrate the camera
  int cameraFlags = cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_PRINCIPAL_POINT +
                    cv::CALIB_FIX_K2 + cv::CALIB_FIX_K3 +
                    cv::CALIB_ZERO_TANGENT_DIST;
  std::vector<cv::Mat> cam_rvecs, cam_tvecs;
  cv::Size frameSize(frameWidth, frameHeight);
  cal.cam_error = cv::calibrateCamera(
      Q, qc, frameSize, cal.Kc, cal.kc, cam_rvecs, cam_tvecs, cal.cam_stdint,
      cal.cam_stdext, cal.cam_pve, cameraFlags,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50,
                       DBL_EPSILON));

  // calibrate the projector
  int projectorFlags = cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_K3;
  std::vector<cv::Mat> proj_rvecs, proj_tvecs;
  cv::Size screenSize(screenCols, screenRows);
  cal.proj_error = cv::calibrateCamera(
      Q, qp, screenSize, cal.Kp, cal.kp, proj_rvecs, proj_tvecs,
      cal.proj_stdint, cal.proj_stdext, cal.proj_pve, projectorFlags,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50,
                       DBL_EPSILON));

  // stereo calibration
  // note: these output arguments have to be cv::Mats (segfault otherwise)
  cv::Mat Rp, Tp, E, F;
#if CV_MAJOR_VERSION < 3
  double stereo_error = cv::stereoCalibrate(
      Q, qc, qp, cal.Kc, cal.kc, cal.Kp, cal.kp, frameSize, Rp, Tp, E, F,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100,
                       DBL_EPSILON),
      cv::CALIB_FIX_INTRINSIC);
#else
  cal.stereo_error = cv::stereoCalibrate(
      Q, qc, qp, cal.Kc, cal.kc, cal.Kp, cal.kp, frameSize, Rp, Tp, E, F,
      cv::CALIB_FIX_INTRINSIC,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100,
                       DBL_EPSILON));
#endif
  cal.Rp = Rp;
  cal.Tp = Tp;

  cal.calibrationDateTime =
      QDateTime::currentDateTime().toString().toStdString();
  cal.frameHeight = frameHeight;
  cal.frameWidth = frameWidth;

  cal.screenResX = screenCols;
  cal.screenResY = screenRows;

  std::stringstream ss;
  cal.print(ss);

  emit logMessage(QString::fromStdString(ss.str()));

#ifdef _DEBUG
  plotResiduals(Q, qp, cal.Kp, cal.kp, proj_rvecs, proj_tvecs);
#endif

  // draw residuals onto images
  const float errorMagnificationFactor = 10.0f;
  for (unsigned int i = 0; i < Q.size(); ++i) {

    std::vector<cv::Point2f> qcProj;
    cv::projectPoints(cv::Mat(Q[i]), cam_rvecs[i], cam_tvecs[i], cal.Kc, cal.kc,
                      qcProj);

    // camera
    for (unsigned int j = 0; j < qcProj.size(); j++) {
      cv::Point2f d = errorMagnificationFactor * (qc[i][j] - qcProj[j]);

      cv::arrowedLine(results[i], qc[i][j], qc[i][j] + d, cv::Scalar(0, 255, 0),
                      2);
    }

    // projector
    std::vector<cv::Point2f> qpProj;
    cv::projectPoints(cv::Mat(Q[i]), proj_rvecs[i], proj_tvecs[i], cal.Kp,
                      cal.kp, qpProj);
    for (unsigned int j = 0; j < qpProj.size(); j++) {
      cv::Point2f d = errorMagnificationFactor * (qp[i][j] - qpProj[j]);

      cv::arrowedLine(results[i], qc[i][j], qc[i][j] + d, cv::Scalar(0, 0, 255),
                      2);
    }

    emit newSequenceResult(results[i], activeSeqs[i], true);
  }

  return true;
}

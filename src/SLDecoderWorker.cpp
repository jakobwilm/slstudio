#include "SLDecoderWorker.h"

#include "CodecFastRatio.h"
#include "CodecGrayCode.h"
#include "CodecPhaseShift2p1.h"
#include "CodecPhaseShift2x3.h"
#include "CodecPhaseShift3.h"
#include "CodecPhaseShift3FastWrap.h"
#include "CodecPhaseShift3Unwrap.h"
#include "CodecPhaseShift4.h"
#include "CodecPhaseShiftDescatter.h"
#include "CodecPhaseShiftMicro.h"
#include "CodecPhaseShiftModulated.h"
#include "CodecPhaseShiftNStep.h"

#include "CalibrationData.h"

#include <QCoreApplication>
#include <QSettings>

#include <opencv2/opencv.hpp>

#include "cvtools.h"

void SLDecoderWorker::setup() {

  // Initialize decoder
  QSettings settings("SLStudio");

  CodecDir dir =
      (CodecDir)settings.value("pattern/direction", CodecDirHorizontal).toInt();
  if (dir == CodecDirNone)
    std::cerr << "SLDecoderWorker: invalid coding direction " << std::endl;
  bool diamondPattern =
      settings.value("projector/diamondPattern", false).toBool();

  CalibrationData calib;
  calib.load("calibration.xml");

  if (diamondPattern) {
    screenCols = 2 * calib.screenResX;
    screenRows = calib.screenResY;
  } else {
    screenCols = calib.screenResX;
    screenRows = calib.screenResY;
  }

  QString patternMode =
      settings.value("pattern/mode", "CodecPhaseShift3").toString();
  if (patternMode == "CodecPhaseShift3")
    decoder = new DecoderPhaseShift3(screenCols, screenRows, dir);
  else if (patternMode == "CodecPhaseShift4")
    decoder = new DecoderPhaseShift4(screenCols, screenRows, dir);
  else if (patternMode == "CodecPhaseShift2x3")
    decoder = new DecoderPhaseShift2x3(screenCols, screenRows, dir);
  else if (patternMode == "CodecPhaseShift3Unwrap")
    decoder = new DecoderPhaseShift3Unwrap(screenCols, screenRows, dir);
  else if (patternMode == "CodecPhaseShiftNStep")
    decoder = new DecoderPhaseShiftNStep(screenCols, screenRows, dir);
  else if (patternMode == "CodecPhaseShift3FastWrap")
    decoder = new DecoderPhaseShift3FastWrap(screenCols, screenRows, dir);
  else if (patternMode == "CodecPhaseShift2p1")
    decoder = new DecoderPhaseShift2p1(screenCols, screenRows, dir);
  else if (patternMode == "CodecPhaseShiftDescatter")
    decoder = new DecoderPhaseShiftDescatter(screenCols, screenRows, dir);
  else if (patternMode == "CodecPhaseShiftModulated")
    decoder = new DecoderPhaseShiftModulated(screenCols, screenRows, dir);
  else if (patternMode == "CodecPhaseShiftMicro")
    decoder = new DecoderPhaseShiftMicro(screenCols, screenRows, dir);
  else if (patternMode == "CodecFastRatio")
    decoder = new DecoderFastRatio(screenCols, screenRows, dir);
  else if (patternMode == "CodecGrayCode")
    decoder = new DecoderGrayCode(screenCols, screenRows, dir);
  else
    std::cerr << "SLDecoderWorker: invalid pattern mode "
              << patternMode.toStdString() << std::endl;

  timer.start();
}

void SLDecoderWorker::decodeSequence(std::vector<cv::Mat> frameSeq) {

  // Recursively call self until latest event is hit
  busy = true;
  QCoreApplication::sendPostedEvents(this, QEvent::MetaCall);
  bool result = busy;
  busy = false;
  if (!result) {
    std::cerr << "SLDecoderWorker: dropped frame sequence!" << std::endl;
    return;
  }

  timer.restart();

  for (unsigned int i = 0; i < frameSeq.size(); i++)
    decoder->setFrame(i, frameSeq[i]);

  // Decode frame sequence
  cv::Mat mask(frameSeq[0].size(), cv::DataType<bool>::type);
  cv::Mat shading(frameSeq[0].size(), CV_8U);

  cv::Mat up, vp;
  if (decoder->getDir() & CodecDirHorizontal)
    up.create(frameSeq[0].size(), CV_32FC1);
  if (decoder->getDir() & CodecDirVertical)
    vp.create(frameSeq[0].size(), CV_32FC1);

  decoder->decodeFrames(up, vp, mask, shading);

  // Emit result
  emit newUpVp(up, vp, mask, shading);

  if (!up.empty()) {
    cv::Mat upMasked;
    cv::add(up * (255.0 / screenCols), 0.0, upMasked, mask);
    upMasked.convertTo(upMasked, CV_8U);
    emit showDecoderUp(upMasked);
  }
  if (!vp.empty()) {
    cv::Mat vpMasked;
    cv::add(vp * (255.0 / screenRows), 0.0, vpMasked, mask);
    vpMasked.convertTo(vpMasked, CV_8U);
    emit showDecoderVp(vpMasked);
  }

  // Emit shading for display in GUI
  emit showShading(shading);

  std::cout << "Decoder: " << timer.restart() << "ms" << std::endl;
}

SLDecoderWorker::~SLDecoderWorker() { delete decoder; }

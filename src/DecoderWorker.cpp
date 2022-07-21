#include "DecoderWorker.h"

#include "CalibrationData.h"
#include "CodecFactory.h"

#include <QCoreApplication>
#include <QSettings>

#include <opencv2/opencv.hpp>

#include "cvtools.h"

void DecoderWorker::setup() {

  // Initialize decoder
  QSettings settings("SLStudio");

  CodecDir dir = static_cast<CodecDir>(
      settings.value("pattern/direction", CodecDirHorizontal).toInt());
  if (dir == CodecDirNone)
    std::cerr << "SLDecoderWorker: invalid coding direction " << std::endl;

  CalibrationData calib;
  calib.load("calibration.xml");

  screenResX = calib.screenResX;
  screenResY = calib.screenResY;

  const std::string codecName =
      settings.value("pattern/mode", "PhaseShift2x3").toString().toStdString();

  if (Codecs.count(codecName) == 0) {
    std::cerr << "DecoderWorker: invalid codec " << codecName << std::endl;
  }
  decoder = DecoderFactory::NewDecoder(Codecs.at(codecName), screenResX,
                                       screenResY, dir);

  timer.start();
}

void DecoderWorker::decodeSequence(std::vector<cv::Mat> frameSeq) {

  // Recursively call self until latest event is hit
  busy = true;
  QCoreApplication::sendPostedEvents(this, QEvent::MetaCall);
  bool result = busy;
  busy = false;
  if (!result) {
    std::cerr << "DecoderWorker: dropped frame sequence!" << std::endl;
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
    cv::add(up * (255.0 / screenResX), 0.0, upMasked, mask);
    upMasked.convertTo(upMasked, CV_8U);
    emit showDecoderUp(upMasked);
  }
  if (!vp.empty()) {
    cv::Mat vpMasked;
    cv::add(vp * (255.0 / screenResY), 0.0, vpMasked, mask);
    vpMasked.convertTo(vpMasked, CV_8U);
    emit showDecoderVp(vpMasked);
  }

  // Emit shading for display in GUI
  emit showShading(shading);

  emit logMessage(QString("Decoder: %1ms").arg(timer.restart()));
}

DecoderWorker::~DecoderWorker() {}

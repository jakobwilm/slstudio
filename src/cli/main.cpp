#include "DecoderWorker.h"
#include "TriangulatorWorker.h"
#include "FrameSequence.h"

#include <iostream>

int main(int argc, char **argv) {
  if (argc < 4 || argc > 5) {
    std::cerr << "Usage: slstudio-reconstruct calibration.xml sequence-folder output.ply [codec]\n";
    return 1;
  }
  try {
    GeometricCalibrationData calibration;
    if (!calibration.load(argv[1])) throw std::runtime_error("Cannot load calibration");
    auto sequence = loadSequence(argv[2]);
    auto settings = withCodecSettings({}, sequence.coding);
    if (argc == 5) settings.coding.codec = codecTypeFromName(argv[4]);
    DecodeStage decoder(calibration, settings);
    TriangulationStage triangulator(calibration, settings);
    PointCloudPtr cloud;
    if (settings.coding.codec == CodecType::GrayCode) {
      cloud = triangulator.triangulateSparse(decoder.decodeTransitions(sequence.frames));
    } else {
      cv::Mat1f up, vp;
      cv::Mat1b mask, shading;
      decoder.decodeMaps(sequence.frames, up, vp, mask, shading);
      cloud = triangulator.triangulateDense(up, vp, mask, shading);
    }
    if (!savePointCloudPLY(argv[3], *cloud)) throw std::runtime_error("Cannot save point cloud");
    std::cout << "Saved " << cloud->points.size() << " points\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
#include "CodecPhaseShiftModulated.h"
#include <cstddef>
#include <iomanip>
#include <math.h>

#include "pstools.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Encoder

#define USE_SINE_MODULATOR 1

EncoderPhaseShiftModulated::EncoderPhaseShiftModulated(unsigned int _screenCols,
                                                       unsigned int _screenRows,
                                                       CodecDir _dir, const PhaseShiftModulatedSettings &settings)
    : Encoder(_screenCols, _screenRows, _dir) {
  CodecAlgorithmSettings requestedSettings;
  requestedSettings.modulated = settings;
  validateCodecSettings(requestedSettings);
  const auto Nx = settings.signalSteps, Ny = settings.carrierSteps, Ncue = settings.cueSteps;
  const auto nPhaseX = settings.signalPeriods, nPhaseY = settings.carrierPeriods;

  // Set N
  // N = Ny * (Nx+Ncue);
  N = Ny * Nx + Ncue;

  // Precompute encoded patterns
  const float pi = M_PI;

  std::vector<cv::Mat> Lx; // Signal
  std::vector<cv::Mat> My; // Carriers

  for (int x = 0; x < Nx; x++) {
    float phaseX = 2 * pi / Nx * x;
    float pitch = (float)screenCols / (float)nPhaseX;
    cv::Mat lx(screenRows, screenCols, CV_32F);
    for (uint r = 0; r < screenRows; r++) {
      for (uint c = 0; c < screenCols; c++) {
        float phase = 2 * pi * c / pitch - phaseX;
        float amp = 0.5 + 0.5 * cos(phase);
        lx.at<float>(r, c) = amp;
      }
    }
    Lx.push_back(lx);
  }

#if USE_SINE_MODULATOR
  for (int y = 0; y < Ny; y++) {
    float phaseY = 2 * pi / Ny * y;
    // float pitch = (float)screenCols / (float)nPhaseY;
    float pitch = (float)screenRows / (float)nPhaseY;
    cv::Mat my(screenRows, screenCols, CV_32F);
    for (uint r = 0; r < screenRows; r++) {
      for (uint c = 0; c < screenCols; c++) {
        float phase = 2 * pi * r / pitch - phaseY;
        // float phase = 2 * pi * c / pitch - phaseY;
        float amp = 0.5 + 0.5 * cos(phase);
        my.at<float>(r, c) = amp;
      }
    }
    My.push_back(my);
  }
#else
  for (uint y = 0; y < Ny; y++) {
    uint period = screenCols / nPhaseY;
    // uint period = screenRows / nPhaseY;
    uint phase = period * y / Ny;
    cv::Mat my(screenRows, screenCols, CV_32F);
    for (uint r = 0; r < screenRows; r++) {
      for (uint c = 0; c < screenCols; c++) {
        float amp = (r + phase) % period > period / 2 ? 1 : 0;
        // float amp = (c + phase) % period > period / 2 ? 1 : 0;
        my.at<float>(r, c) = amp;
      }
    }
    My.push_back(my);
  }
#endif
  for (uint x = 0; x < Lx.size(); x++) {
    for (uint y = 0; y < My.size(); y++) {
      cv::Mat I;
      cv::multiply(Lx[x], My[y], I, 255, CV_8U);
      cv::cvtColor(I, I, cv::COLOR_GRAY2RGB);
      patterns.push_back(I);
    }
  }

  // Phase cue patterns
  for (int i = 0; i < Ncue; i++) {
    float phase = 2.0 * pi / Ncue * i;
    float pitch = screenCols;
    cv::Mat patternI;
    patternI = pstools::computePhaseVector(screenCols, phase, pitch);
    patternI = patternI.t();
    patterns.push_back(patternI);
  }

#if 0
        for(unsigned int i=0; i<patterns.size(); i++){
            std::stringstream fileNameStream;
            fileNameStream << "pattern_" << std::setw(2) << std::setfill('0') << i << ".bmp";
            cv::imwrite(fileNameStream.str(), patterns[i]);
        }

#endif
}

cv::Mat EncoderPhaseShiftModulated::getEncodingPattern(unsigned int depth) const {
  return patterns.at(depth);
}

// Decoder
DecoderPhaseShiftModulated::DecoderPhaseShiftModulated(unsigned int _screenCols,
                                                       unsigned int _screenRows,
                                                       CodecDir _dir, const PhaseShiftModulatedSettings &requestedSettings)
    : Decoder(_screenCols, _screenRows, _dir), settings(requestedSettings) {
  CodecAlgorithmSettings algorithmSettings;
  algorithmSettings.modulated = settings;
  validateCodecSettings(algorithmSettings);

  // N = (Nx+Ncue) * Ny;
  N = settings.carrierSteps * settings.signalSteps + settings.cueSteps;
}

void DecoderPhaseShiftModulated::decodeFrames(const std::vector<cv::Mat> &frames, cv::Mat &up,
                                              cv::Mat &, cv::Mat &mask, cv::Mat &shading) {
  validateCodecFrames(frames, N);

  const float pi = M_PI;
  const auto Nx = settings.signalSteps, Ny = settings.carrierSteps, Ncue = settings.cueSteps;
  const auto nPhaseX = settings.signalPeriods;

  std::vector<cv::Mat> framesX;

  // Decoding
#if USE_SINE_MODULATOR
  for (std::size_t x = Ny; x <= frames.size() - Ncue; x += Ny) {

    std::vector<cv::Mat> framesY(frames.begin() + (x - Ny), frames.begin() + x);
    std::vector<cv::Mat> fIcomp = pstools::getDFTComponents(framesY);

    cv::Mat frameX;
    cv::magnitude(fIcomp[2], fIcomp[3], frameX);
    framesX.push_back(frameX);
  }
#else
  for (std::size_t x = Ny; x <= frames.size() - Ncue; x += Ny) {

    std::vector<cv::Mat> framesY(frames.begin() + (x - Ny), frames.begin() + x);
    cv::Mat Imin = cv::min(framesY[0], framesY[1]);
    cv::Mat Imax = cv::max(framesY[0], framesY[1]);
    for (uint i = 2; i < framesY.size(); i++) {
      Imin = cv::min(Imin, framesY[i]);
      Imax = cv::max(Imax, framesY[i]);
    }
    cv::Mat Ld = Imax - Imin;
    framesX.push_back(Ld);
  }
#endif
  std::vector<cv::Mat> framesPhase(framesX.begin(), framesX.end());
  std::vector<cv::Mat> fIcomp = pstools::getDFTComponents(framesPhase);
  cv::phase(fIcomp[2], -fIcomp[3], up);

  // Calculate modulation
  cv::magnitude(fIcomp[2], fIcomp[3], shading);
  shading.convertTo(shading, CV_8U, 6.0 / (Nx * Ny));

  std::vector<cv::Mat> framesCue(frames.end() - Ncue, frames.end());
  fIcomp = pstools::getDFTComponents(framesCue);
  cv::Mat upCue;
  cv::phase(fIcomp[2], -fIcomp[3], upCue);

  up = pstools::unwrapWithCue(up, upCue, nPhaseX);
  up *= screenCols / (2 * pi);

  // Threshold modulation image for mask
  mask = shading > modulationThreshold(20);
}

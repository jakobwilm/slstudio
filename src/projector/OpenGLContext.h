//
//  OpenGLContext.h
//  OpenGLContext
//
//  Created by Jakob Wilm on 18/03/13.
//  Copyright (c) 2013 Jakob Wilm. All rights reserved.
//

#pragma once

#include <iostream>
#include <sys/types.h>
#include <vector>

struct ScreenInfo {
  unsigned int resX, resY;
  unsigned int posX, posY;
  std::string name;
  ScreenInfo() : resX(0), resY(0), posX(0), posY(0) {}
};

// Platform dependent OpenGLContext class
class OpenGLContext {
public:
  // Static "class" methods
  static std::vector<ScreenInfo> GetScreenInfo();
  // Interface function
  OpenGLContext(unsigned int _screenNum = 0);
  void setGamma(float gamma);
  unsigned int getScreenResX() { return screenResX; }
  unsigned int getScreenResY() { return screenResY; }
  ~OpenGLContext();
  void makeContextCurrent();
  void flush();

private:
  unsigned int screenNum;
  unsigned int screenResX, screenResY;

  // Opaque data type defined in platform specific source file
  struct OpenGLContextInfo;
  OpenGLContextInfo *contextInfo;
};

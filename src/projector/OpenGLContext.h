//
//  OpenGLContext.h
//  OpenGLContext
//
//  Created by Jakob Wilm on 18/03/13.
//  Copyright (c) 2013 Jakob Wilm. All rights reserved.
//

#ifndef OPENGLCONTEXT_H
#define OPENGLCONTEXT_H

#include <iostream>
#include <vector>
#include <sys/types.h>

#include "Projector.h"

// Platform dependent OpenGLContext class
class OpenGLContext{
    public:
        // Static "class" methods
        static std::vector<ScreenInfo> GetScreenInfo();
        // Interface function
        OpenGLContext(unsigned int _screenNum = 0);
        void setGamma(float gamma);
        unsigned int getScreenResX(){return screenResX;}
        unsigned int getScreenResY(){return screenResY;}
        ~OpenGLContext();
        void makeContextCurrent();
        void flush();
    private:
        unsigned int screenNum;
        unsigned int screenResX, screenResY;
        // Opaque data type defined in platform specific files
        struct OpenGLContextInfo;
        OpenGLContextInfo *contextInfo;
};

#endif

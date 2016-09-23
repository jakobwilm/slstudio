#ifndef PROJECTOROPENGL_H
#define PROJECTOROPENGL_H

#include <iostream>
#include <vector>

#include <GL/glew.h>
#include "Projector.h"

class OpenGLContext;

// ProjectorOpenGL implementations
class ProjectorOpenGL : public Projector {
    public:
        // Interface function
        ProjectorOpenGL(unsigned int _screenNum = 0);
        // Define preset pattern sequence and upload to GPU
        void setPattern(unsigned int patternNumber, const unsigned char *tex, unsigned int texWidth, unsigned int texHeight);
        void displayPattern(unsigned int patternNumber);
        // Upload and display pattern on the fly
        void displayTexture(const unsigned char *tex, unsigned int width, unsigned int height);
        void displayBlack();
        void displayWhite();
        void getScreenRes(unsigned int *nx, unsigned int *ny);
        ~ProjectorOpenGL();
    private:
        std::vector<GLuint> frameBuffers;
        OpenGLContext *context;
        GLuint shaderProgram;
};

#endif

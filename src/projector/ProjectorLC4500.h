#ifndef PROJECTORLC4500_H
#define PROJECTORLC4500_H

#include <iostream>
#include <vector>
#include <sys/types.h>

#include "Projector.h"

// Projecotr implementation for LightCrafter 4500 USB Api
class ProjectorLC4500 : public Projector {
    public:
        // Interface function
        ProjectorLC4500(unsigned int);
        // Define preset pattern sequence and upload to GPU
        void setPattern(unsigned int patternNumber, const unsigned char *tex, unsigned int texWidth, unsigned int texHeight);
        void displayPattern(unsigned int patternNumber);
        // Upload and display pattern on the fly
        void displayTexture(const unsigned char *tex, unsigned int width, unsigned int height);
        void displayBlack();
        void displayWhite();
        void getScreenRes(unsigned int *nx, unsigned int *ny);
        ~ProjectorLC4500();
    private:
        unsigned int nPatterns;
        bool isRunning;
};

#endif

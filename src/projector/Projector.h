#ifndef PROJECTOR_H
#define PROJECTOR_H

#include <iostream>
#include <vector>

// Abstract Projector base class
class Projector {
    public:
        // Interface function
        Projector(){}
        // Define preset pattern sequence
        virtual void setPattern(unsigned int patternNumber, const unsigned char *tex, unsigned int texWidth, unsigned int texHeight) = 0;
        virtual void displayPattern(unsigned int patternNumber) = 0;
        // Upload and display pattern on the fly
        virtual void displayTexture(const unsigned char *tex, unsigned int width, unsigned int height) = 0;
        // Monochrome color display
        virtual void displayBlack() = 0;
        virtual void displayWhite() = 0;
        virtual void getScreenRes(unsigned int *nx, unsigned int *ny) = 0;
        virtual ~Projector(){}
};

#endif

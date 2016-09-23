#ifndef PROJECTOR_H
#define PROJECTOR_H

#include <iostream>
#include <set>
#include <vector>

#include "slprojector_export.h"

enum ProjectorType {
    projectorTypeOpenGL,
    projectorTypeLC3000,
    projectorTypeLC4500,
    projectorTypeQtGL
};

struct ScreenInfo {
    unsigned int resX, resY;
    unsigned int posX, posY;
    std::string name;
    ScreenInfo(): resX(0), resY(0), posX(0), posY(0){}
};

// Abstract Projector base class
class SLPROJECTOR_EXPORT Projector {
public:
    static std::set<ProjectorType> GetProjectorList();
    static Projector* NewProjector(ProjectorType projector, unsigned int screenNum = 0);
    static std::vector<ScreenInfo> GetScreenInfo();

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

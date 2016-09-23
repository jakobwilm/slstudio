#ifndef PROJECTORLC3000_H
#define PROJECTORLC3000_H

#include <iostream>
#include <vector>
#include <sys/types.h>

#include "Projector.h"


#include "LC3000API/lcr_cmd.h"

// Projector implementation for LightCrafter 3000 USB Api
class ProjectorLC3000 : public Projector {
    public:
        // Interface function
        ProjectorLC3000(unsigned int);
        // Define preset pattern sequence and upload to GPU
        void setPattern(unsigned int patternNumber, const unsigned char *tex, unsigned int texWidth, unsigned int texHeight);
        void displayPattern(unsigned int patternNumber);
        // Upload and display pattern on the fly
        void displayTexture(const unsigned char *tex, unsigned int width, unsigned int height);
        void displayBlack();
        void displayWhite();
        void getScreenRes(unsigned int *nx, unsigned int *ny);
        ~ProjectorLC3000();
        bool ptn_seq_mode;
    private:
        LCR_PatternSeqSetting_t patternSeqSettings;
        ErrorCode_t res;
};

#endif

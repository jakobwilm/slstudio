//
//  main.cpp
//  patternGenerator
//
//  Created by Jakob Wilm on 18/03/13.
//  Copyright (c) 2013 Jakob Wilm. All rights reserved.
//

#include <iostream>
//#include <unistd.h>
#include "Projector.h"
#include "ProjectorGrayCode.h"
#ifdef _WIN32
    #define usleep(x) Sleep(x)
#endif
int main(int argc, char** argv){

    std::vector<ScreenInfo> screenInfo;
    screenInfo = Projector::GetScreenInfo();
    
    for (int i=0; i<screenInfo.size(); i++) {
        printf("Screen %d: width %d, height %d\n", i, screenInfo[i].resX, screenInfo[i].resY);
    }
    
//    Projector *PP = new Projector(0);
    
//    unsigned char checkImage[200][200][3];
//    int i, j;
//    for (int k=0; k<3; k++) {
//        for (i = 0; i < 200; i++) {
//            for (j = 0; j < 101; j++) {
//                checkImage[i][j][k] = 50;
//            }
//            for (j = 101; j < 200; j++) {
//                checkImage[i][j][k] = 100;
//            }
//        }
//    }
    
//    PP->DisplayTexture((unsigned char*)checkImage, 200, 200);
    
    ProjectorGrayCode *GCP = new ProjectorGrayCode(1);

    for(unsigned int depth=0; depth<15; depth++){
        GCP->displayPattern(depth);
        usleep(50000);
     }
    
    delete GCP;
    return 0;
}


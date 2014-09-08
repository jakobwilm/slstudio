#include "mex.h"
#include "class_handle.hpp"

// Output printf statements in the Matlab workspace
#define printf mexPrintf


// The class that we are interfacing to
#include "ProjectorOpenGL.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{	
    // Get the command string
    char cmd[64];
	if (nrhs < 1 || mxGetString(prhs[0], cmd, sizeof(cmd)))
		mexErrMsgTxt("First input should be a command string less than 64 characters long.");
    
    // Static methods
    // DisplayInfo
    if (!strcmp("GetScreenInfo", cmd)) {
        // Call the method
        std::vector<ScreenInfo> screenInfo;
        screenInfo = OpenGLContext::GetScreenInfo();
        const char* fieldNames[] = {"screenNum","resX","resY"};
        plhs[0] = mxCreateStructMatrix(1, screenInfo.size(), 3, fieldNames);
        for (unsigned int i=0; i<screenInfo.size(); i++) {
            mxSetFieldByNumber(plhs[0], i, 0, mxCreateDoubleScalar(i));
            mxSetFieldByNumber(plhs[0], i, 1, mxCreateDoubleScalar(screenInfo[i].resX));
            mxSetFieldByNumber(plhs[0], i, 2, mxCreateDoubleScalar(screenInfo[i].resY));
        }
        return;
    }
    
    // New
    if (!strcmp("new", cmd)) {
        // Check parameters
        if (nlhs != 1)
            mexErrMsgTxt("New: One output expected.");
        if (nrhs < 2)
            mexErrMsgTxt("New: Expected screen number argument.");
        unsigned int screenNum = (unsigned int)mxGetScalar(prhs[1]);
        // Return a handle to a new C++ instance
        plhs[0] = convertPtr2Mat<Projector>(new ProjectorOpenGL(screenNum));
        return;
    }
    
    // Check if there is a second input, which should be the class instance handle
    if (nrhs < 2)
		mexErrMsgTxt("Second input should be a class instance handle.");
    
    // Delete
    if (!strcmp("delete", cmd)) {
        // Destroy the C++ object
        destroyObject<Projector>(prhs[1]);
        // Warn if other commands were ignored
        if (nlhs != 0 || nrhs != 2)
            mexWarnMsgTxt("Delete: Unexpected arguments ignored.");
        return;
    }
    
    // Get the class instance pointer from the second input
    Projector *Projector_instance = convertMat2Ptr<Projector>(prhs[1]);
    
    // Call the various class methods
    // DisplayTexture
    if (!strcmp("displayTexture", cmd)) {
        // Check parameters
        if (nlhs < 0 || nrhs < 5)
            mexErrMsgTxt("DisplayTexture: Unexpected arguments.");
        // Call the method
        unsigned char *tex = (unsigned char*)mxGetData(prhs[2]);
        unsigned int width = (unsigned int)mxGetScalar(prhs[3]);
        unsigned int height = (unsigned int)mxGetScalar(prhs[4]);
        Projector_instance->displayTexture(tex, width, height);
        return;
    }
   // DisplayWhite
   if (!strcmp("displayWhite", cmd)) {
       // Check parameters
       if (nlhs < 0 || nrhs < 1)
           mexErrMsgTxt("Test: Unexpected arguments.");
       // Call the method
       Projector_instance->displayWhite();
       return;
   }
   // DisplayBlack
   if (!strcmp("displayBlack", cmd)) {
       // Check parameters
       if (nlhs < 0 || nrhs < 1)
           mexErrMsgTxt("Test: Unexpected arguments.");
       // Call the method
       Projector_instance->displayBlack();
       return;
   }    
    // Got here, so command not recognized
    mexErrMsgTxt("Command not recognized.");
}

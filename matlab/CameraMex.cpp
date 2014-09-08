#include "mex.h"
#include "class_handle.hpp"

// Output printf statements in the Matlab workspace
#define printf mexPrintf

// The class that we are interfacing to
#include "Camera.h"

using namespace std;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{	
    // Get the command string
    char cmd[64];
	if (nrhs < 1 || mxGetString(prhs[0], cmd, sizeof(cmd)))
		mexErrMsgTxt("First input should be a command string less than 64 characters long.");
    
    // Static methods
    // InterfaceCameraList
    if (!strcmp("GetInterfaceCameraList", cmd)) {
        // Call the method
        vector< vector<CameraInfo> > interfaceCameraList;
        interfaceCameraList = Camera::GetInterfaceCameraList();
        unsigned int nCameras = 0;
        for(unsigned int i=0; i<interfaceCameraList.size(); i++)
            nCameras += interfaceCameraList[i].size();
        const char* fieldNames[] = {"interfaceNum", "cameraNum", "vendor","model"};
        plhs[0] = mxCreateStructMatrix(1, nCameras, 4, fieldNames);
        unsigned int iCamera = 0;
        for (unsigned int i=0; i<interfaceCameraList.size(); i++) {
            for (unsigned int j=0; j<interfaceCameraList[i].size(); j++) {
                mxSetFieldByNumber(plhs[0], iCamera, 0, mxCreateDoubleScalar(i));
                mxSetFieldByNumber(plhs[0], iCamera, 1, mxCreateDoubleScalar(j));
                mxSetFieldByNumber(plhs[0], iCamera, 2, mxCreateString(interfaceCameraList[i][j].vendor.c_str()));
                mxSetFieldByNumber(plhs[0], iCamera, 3, mxCreateString(interfaceCameraList[i][j].model.c_str()));
                iCamera++;
            }
        }
        return;
    }
    
    // New Camera
    if (!strcmp("NewCamera", cmd)) {
        // Check parameters
        if (nlhs != 1)
            mexErrMsgTxt("NewCamera: One output expected.");
        if (nrhs < 3)
            mexErrMsgTxt("NewCamera: Expected interface, screen number and trigger mode argument.");
        unsigned int interfaceNum = (unsigned int)mxGetScalar(prhs[1]);
        unsigned int camNum = (unsigned int)mxGetScalar(prhs[2]);
            
        // Return a handle to a new C++ instance
        plhs[0] = convertPtr2Mat<Camera>(Camera::NewCamera(interfaceNum, camNum, triggerModeSoftware));
        return;
    }
    
    // Class methods
    // Check if there is a second input, which should be the class instance handle
    if (nrhs < 2)
		mexErrMsgTxt("Second input should be a class instance handle.");
    
    // Delete
    if (!strcmp("delete", cmd)) {
        // Destroy the C++ object
        destroyObject<Camera>(prhs[1]);
        // Warn if other commands were ignored
        if (nlhs != 0 || nrhs != 2)
            mexWarnMsgTxt("Delete: Unexpected arguments ignored.");
        return;
    }
    
    // Get the class instance pointer from the second input
    Camera *Camera_instance = convertMat2Ptr<Camera>(prhs[1]);
    
    // Call the various class methods
    
    // startCapture
    if (!strcmp("startCapture", cmd)) {
        // Check parameters
        if (nrhs > 2)
            mexErrMsgTxt("startCapture: Unexpected arguments.");
        // Call the method
        Camera_instance->startCapture();
        return;
    }
    
	// stopCapture
    if (!strcmp("stopCapture", cmd)) {
        // Check parameters
        if (nrhs > 2)
            mexErrMsgTxt("stopCapture: Unexpected arguments.");
        // Call the method
        Camera_instance->stopCapture();
        return;
    }
    
    // getFrame
    if (!strcmp("getFrame", cmd)) {
        // Check parameters
        if (nlhs < 1 || nrhs > 2)
            mexErrMsgTxt("getFrame: Unexpected arguments.");
        // Call the method
        CameraFrame frame = Camera_instance->getFrame();
        plhs[0] = mxCreateNumericMatrix(frame.width, frame.height, mxUINT8_CLASS, mxREAL);
        memcpy(mxGetData(plhs[0]), frame.memory, frame.height*frame.width);
        //Camera_instance->unlockFrame();
        return;
    }

//    // Test
//    if (!strcmp("test", cmd)) {
//        // Check parameters
//        if (nlhs < 0 || nrhs < 2)
//            mexErrMsgTxt("Test: Unexpected arguments.");
//        // Call the method
//        PatternProjector_instance->test();
//        return;
//    }
    
    // Got here, so command not recognized
    mexErrMsgTxt("Command not recognized.");
}

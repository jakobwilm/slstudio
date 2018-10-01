#include "Camera.h"

#ifdef WITH_CAMERACV
    #include "CameraCV.h"
#endif
#ifdef WITH_CAMERAV4L2
    #include "CameraV4L2.h"
#endif
#ifdef WITH_CAMERAIIDC
    #include "CameraIIDC.h"
#endif
#ifdef WITH_CAMERAXIMEA
    #include "CameraXIMEA.h"
#endif
#ifdef WITH_CAMERAIDSIMAGING
    #include "CameraIDSImaging.h"
#endif
#ifdef WITH_CAMERAPOINTGREY
    #include "CameraPointGrey.h"
#endif

// Global camera enumerator
std::vector< std::vector<CameraInfo> > Camera::GetInterfaceCameraList(){
    std::vector< std::vector<CameraInfo> > ret;

#ifdef WITH_CAMERACV
    std::vector<CameraInfo> cvcameras = CameraCV::getCameraList();
    ret.push_back(cvcameras);
#endif
#ifdef WITH_CAMERAV4L2
    std::vector<CameraInfo> v4l2cameras = CameraV4L2::getCameraList();
    ret.push_back(v4l2cameras);
#endif
#ifdef WITH_CAMERAIIDC
    std::vector<CameraInfo> iidccameras = CameraIIDC::getCameraList();
    ret.push_back(iidccameras);
#endif
#ifdef WITH_CAMERAXIMEA
    std::vector<CameraInfo> ximeacameras = CameraXIMEA::getCameraList();
    ret.push_back(ximeacameras);
#endif
#ifdef WITH_CAMERAIDSIMAGING
    std::vector<CameraInfo> idscameras = CameraIDSImaging::getCameraList();
    ret.push_back(idscameras);
#endif
#ifdef WITH_CAMERAPOINTGREY
    std::vector<CameraInfo> ptgreycameras = CameraPointGrey::getCameraList();
    ret.push_back(ptgreycameras);
#endif

    return ret;
}

// Camera factory
Camera* Camera::NewCamera(unsigned int interfaceNum, unsigned int camNum, CameraTriggerMode triggerMode){

    interfaceNum += 1;

#ifdef WITH_CAMERACV
    interfaceNum -= 1;
    if(interfaceNum == 0)
        return new CameraCV(camNum, triggerMode);
#endif
#ifdef WITH_CAMERAV4L2
    interfaceNum -= 1;
    if(interfaceNum == 0)
        return new CameraV4L2(camNum, triggerMode);
#endif
#ifdef WITH_CAMERAIIDC
    interfaceNum -= 1;
    if(interfaceNum == 0)
        return new CameraIIDC(camNum, triggerMode);
#endif
#ifdef WITH_CAMERAXIMEA
    interfaceNum -= 1;
    if(interfaceNum == 0)
        return new CameraXIMEA(camNum, triggerMode);
#endif
#ifdef WITH_CAMERAIDSIMAGING
    interfaceNum -= 1;
    if(interfaceNum == 0)
        return new CameraIDSImaging(camNum, triggerMode);
#endif
#ifdef WITH_CAMERAPOINTGREY
    interfaceNum -= 1;
    if(interfaceNum == 0)
        return new CameraPointGrey(camNum, triggerMode);
#endif

    return (Camera*)NULL;
}

#include "OpenGLContext.h"
#include <GLFW/glfw3.h>

#ifdef __unix__
    #define GLFW_EXPOSE_NATIVE_X11
    #define GLFW_EXPOSE_NATIVE_GLX
    #include <cstring>
    #include <GLFW/glfw3native.h>
#endif

struct OpenGLContext::OpenGLContextInfo{
    GLFWmonitor* monitor;
    GLFWwindow* window;
    OpenGLContextInfo() : monitor(NULL), window(NULL){}
};

void error_callback(int error, const char* description){
    std::cerr << "GLFW error code " << error << " " << description << std::endl;
}


std::vector<ScreenInfo> OpenGLContext::GetScreenInfo(){
    std::vector<ScreenInfo> ret;

    if (!glfwInit())
        std::cerr << "Could not initialize GLFW!" << std::endl;
    glfwSetErrorCallback(error_callback);

    int nScreens = 0;
    GLFWmonitor **monitors;
    monitors = glfwGetMonitors(&nScreens);

    for(int i=0; i<nScreens; i++){

        GLFWmonitor *monitor = monitors[i];
        const GLFWvidmode *videoMode = glfwGetVideoMode(monitor);

        ScreenInfo screen;
        screen.resX = videoMode->width;
        screen.resY = videoMode->height;
        const char *screenName = glfwGetMonitorName(monitor);
        screen.name = screenName;

        ret.push_back(screen);
    }

    //glfwTerminate();

    return ret;
}

OpenGLContext::OpenGLContext(uint _screenNum) : screenNum(_screenNum), screenResX(0), screenResY(0){

    contextInfo = new OpenGLContextInfo();

    if (!glfwInit())
        std::cerr << "Could not initialize GLFW!" << std::endl;
    glfwSetErrorCallback(error_callback);

    int nScreens = 0;
    GLFWmonitor **monitors;
    monitors = glfwGetMonitors(&nScreens);

    if((unsigned int)nScreens < screenNum+1)
        std::cerr << "Could not open screen " << screenNum << std::endl;

    contextInfo->monitor = monitors[screenNum];
    const GLFWvidmode *videoMode = glfwGetVideoMode(contextInfo->monitor);

    screenResX = videoMode->width;
    screenResY = videoMode->height;

    // Create fullscreen window
    contextInfo->window = glfwCreateWindow(screenResX, screenResY, "GLFW OpenGL Context", contextInfo->monitor, NULL);

#ifdef __unix__
    Display *display = glfwGetX11Display();
    Window window = glfwGetX11Window(contextInfo->window);

//    // Set swap interval to 1 for standard vsync
//    typedef GLvoid (*glXSwapIntervalSGIFunc) (GLint);
//    const char *glx_extensions = glXQueryExtensionsString(display, screenNum);
//    if (strstr(glx_extensions, "GLX_SGI_swap_control")) {
//        PFNGLXSWAPINTERVALSGIPROC SwapIntervalSGI = (PFNGLXSWAPINTERVALSGIPROC)glXGetProcAddressARB((const GLubyte*)"glXSwapIntervalSGI");
//        SwapIntervalSGI(1);
//    } else if (strstr(glx_extensions, "GLX_EXT_swap_control")) {
        PFNGLXSWAPINTERVALEXTPROC SwapIntervalEXT = (PFNGLXSWAPINTERVALEXTPROC)glXGetProcAddressARB((const GLubyte*)"glXSwapIntervalEXT");
        SwapIntervalEXT(display, window, 1);
//    } else {
//        std::cerr << "OpenGLContext.Unix Error: Could not access swap interval extension!" << std::endl;
//    }
#endif

    glfwMakeContextCurrent(contextInfo->window);
    // Set vsync (swap interval 1)
    //glfwSwapInterval(1);

    // Adjust gamma to one
    this->setGamma(1.0);
}

void OpenGLContext::setGamma(float gamma){
    glfwSetGamma(contextInfo->monitor, gamma);
}

void OpenGLContext::makeContextCurrent(){
    glfwMakeContextCurrent(contextInfo->window);
}

void OpenGLContext::flush(){

    // Swap buffers
    glfwSwapBuffers(contextInfo->window);

    // Poll events in order not to lock up
    //glfwPollEvents();
    //glfwWaitEvents();
    //glFinish();
}

OpenGLContext::~OpenGLContext(){

    glfwDestroyWindow(contextInfo->window);
    glfwTerminate();

    delete contextInfo;
}


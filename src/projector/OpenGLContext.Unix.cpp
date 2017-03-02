#include "OpenGLContext.h"

#include <GL/gl.h>
#include <GL/glx.h>
#include <X11/Xlib.h>
#include <stdio.h>
#include <string.h>

#include <unistd.h>

// vmode extension for mode settings, gamma control, etc.
//#include <X11/extensions/xf86vmode.h>

// XRandR extension for virtual screens, mode setting, etc.
//#include <X11/extensions/Xrandr.h>

struct OpenGLContext::OpenGLContextInfo{
    Display *display;
    Window window;
    GLXContext context;
    OpenGLContextInfo() : display(NULL), window(0), context(NULL){}
};

std::vector<ScreenInfo> OpenGLContext::GetScreenInfo(){
    std::vector<ScreenInfo> ret;

    // Connection to default X Server
    Display *display = XOpenDisplay(NULL);

    unsigned int nScreens = ScreenCount(display);

    for(unsigned int i=0; i<nScreens; i++){
        Screen *XScreen = ScreenOfDisplay(display, i);
        ScreenInfo screen;
        screen.resX = XScreen->width;
        screen.resY = XScreen->height;
        char buff[100];
        sprintf(buff, "XScreen no. %d", i);
        screen.name = buff;

        ret.push_back(screen);
    }

    XCloseDisplay(display);

    return ret;
}

OpenGLContext::OpenGLContext(uint _screenNum) : screenNum(_screenNum){

    contextInfo = new OpenGLContextInfo();

    contextInfo->display = XOpenDisplay(NULL);

    if((int)screenNum+1 > ScreenCount(contextInfo->display))
        throw "Could not create OpenGLContext. Screen not available!";

    Screen *xScreen = ScreenOfDisplay(contextInfo->display, screenNum);

    screenResX = xScreen->width;
    screenResY = xScreen->height;

    // Create a OpenGL OpenGLContext on the specified X screen
    contextInfo->window = RootWindow(contextInfo->display, screenNum);

    int attrListDbl[] = {
        GLX_RGBA, GLX_DOUBLEBUFFER,
        GLX_RED_SIZE, 0,
        GLX_GREEN_SIZE, 0,
        GLX_BLUE_SIZE, 0,
        GLX_ALPHA_SIZE, 0,
        GLX_DEPTH_SIZE, 0,
        GLX_STENCIL_SIZE, 0,
        None
    };

    XVisualInfo *visualInfo = glXChooseVisual(contextInfo->display, screenNum, attrListDbl);

    int glxMajor, glxMinor = 0;
    glXQueryVersion(contextInfo->display, &glxMajor, &glxMinor);
    std::cout << "GLX-Version " << glxMajor << "." << glxMinor << std::endl;

    int glMajor, glMinor = 0;
    glGetIntegerv(GL_MAJOR_VERSION, &glMajor);
    glGetIntegerv(GL_MINOR_VERSION, &glMinor);
    std::cout << "GL-Version " << glMajor << "." << glMinor << std::endl;

    // Create a GLX OpenGLContext
    bool directRendering = GL_TRUE;
    contextInfo->context = glXCreateContext(contextInfo->display, visualInfo,  NULL, directRendering);

    // Create colormap
    Colormap colormap = XCreateColormap(contextInfo->display,contextInfo->window,visualInfo->visual,AllocNone);

    // Create the actual window
    unsigned long wamask = CWColormap;

    XSetWindowAttributes wa;
    wa.colormap = colormap;
    wa.border_pixel = 50;
    wa.event_mask = 0;

    // bypass window manager (does not have effect)
    wa.override_redirect = True;

    // show no cursor
    wa.cursor = 0;

    contextInfo->window = XCreateWindow(contextInfo->display, contextInfo->window, 0, 0, screenResX, screenResY, 0, visualInfo->depth, InputOutput, visualInfo->visual, wamask, &wa);

    if(!contextInfo->window)
        std::cerr << "Failed to create X window!" << std::endl;

    // bypass window manager (actually effects here)
    XSetWindowAttributes attributes;
    attributes.override_redirect = True;
    XChangeWindowAttributes(contextInfo->display, contextInfo->window, CWOverrideRedirect, &attributes);

    // Raise window (necessary)
    XMapWindow(contextInfo->display, contextInfo->window);

    // Connect the glx-OpenGLContext to the window
    glXMakeCurrent(contextInfo->display, contextInfo->window, contextInfo->context);

    //XSaveOpenGLContext
    XFlush(contextInfo->display);

    // Check if OpenGLContext is direct
    if (glXIsDirect(contextInfo->display, contextInfo->context))
        std::cout << "OpenGLContext is direct\n";
    else
        std::cout << "OpenGLContext is not direct\n";

    // Set swap interval to 1 for standard vsync
    typedef GLvoid (*glXSwapIntervalSGIFunc) (GLint);
    const char *glx_extensions = glXQueryExtensionsString(contextInfo->display, screenNum);
    if (strstr(glx_extensions, "GLX_SGI_swap_control")) {
        PFNGLXSWAPINTERVALSGIPROC SwapIntervalSGI = (PFNGLXSWAPINTERVALSGIPROC)glXGetProcAddressARB((const GLubyte*)"glXSwapIntervalSGI");
        SwapIntervalSGI(1);
    } else if (strstr(glx_extensions, "GLX_EXT_swap_control")) {
        PFNGLXSWAPINTERVALEXTPROC SwapIntervalEXT = (PFNGLXSWAPINTERVALEXTPROC)glXGetProcAddressARB((const GLubyte*)"glXSwapIntervalEXT");
        SwapIntervalEXT(contextInfo->display, contextInfo->window, 1);
    } else {
        std::cerr << "OpenGLContext.Unix Error: Could not access swap interval extension!" << std::endl;
    }

    // Adjust gamma to one
    //setGamma(1.0);
}

void OpenGLContext::setGamma(float gamma){
    // Adjust gamma
//    XF86VidModeGamma xf86Gamma = {gamma, gamma, gamma};
//    XF86VidModeSetGamma(contextInfo->display, screenNum, &xf86Gamma);
}

void OpenGLContext::makeContextCurrent(){
    glXMakeCurrent(contextInfo->display, contextInfo->window, contextInfo->context);
}

void OpenGLContext::flush(){

    // Swap buffers
    glXSwapBuffers(contextInfo->display, contextInfo->window);

    // Synchronize CPU with vsync buffer swap
    //glFinish();
    //glXWaitGL();

}

OpenGLContext::~OpenGLContext(){
    std::cout<<"Releasing OpenGL Context\n"<<std::flush;
    if(contextInfo->context){
        // Release context (None, NULL)
        if(!glXMakeCurrent(contextInfo->display, None, NULL))
            std::cerr << "Error. Could not release drawing OpenGLContext." << std::endl;

        glXDestroyContext(contextInfo->display, contextInfo->context);
        XCloseDisplay(contextInfo->display);
        delete contextInfo;
    }
}


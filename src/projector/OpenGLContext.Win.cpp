#include "OpenGLContext.h"

#include <windows.h>
#include <winuser.h>
#include <tchar.h>
#include <ShObjIdl.h>
#include <GL/gl.h>

#include <stdio.h>

BOOL CALLBACK monitorEnumerator(HMONITOR hMonitor, HDC, LPRECT, LPARAM dwData){

    MONITORINFOEX monitorInfo;
    ZeroMemory(&monitorInfo, sizeof(MONITORINFOEX));
    monitorInfo.cbSize = sizeof(MONITORINFOEX);

    GetMonitorInfo(hMonitor, &monitorInfo);

    ScreenInfo screen;
    char deviceName[32];
    wcstombs((char *)deviceName, (wchar_t*)monitorInfo.szDevice, 32);
    screen.name = std::string(deviceName);

    RECT rect = monitorInfo.rcMonitor;
    screen.resX = rect.right - rect.left;
    screen.resY = rect.bottom - rect.top;

    screen.posX = rect.left;
    screen.posY = rect.top;

    std::vector<ScreenInfo> *ret = reinterpret_cast<std::vector<ScreenInfo>*>(dwData);
    ret->push_back(screen);

    return true;
}


std::vector<ScreenInfo> OpenGLContext::GetScreenInfo(){

    std::vector<ScreenInfo> ret;

    EnumDisplayMonitors(NULL, NULL, monitorEnumerator, (LPARAM)&ret);


//    DISPLAY_DEVICE dd;
//    ZeroMemory(&dd, sizeof(DISPLAY_DEVICE));
//    dd.cb = sizeof(dd);

//    for(int i=0; EnumDisplayDevices(NULL, i, &dd, 0); i++){

//        if ((dd.StateFlags & DISPLAY_DEVICE_MIRRORING_DRIVER) || !(dd.StateFlags & DISPLAY_DEVICE_ACTIVE))
//            continue;

//        // Get additional info
//        DISPLAY_DEVICE display;
//        ZeroMemory(&display, sizeof(DISPLAY_DEVICE));
//        display.cb = sizeof(DISPLAY_DEVICE);

//        EnumDisplayDevices(dd.DeviceName, 0, &display, 0);
//        HDC dc = CreateDC(L"DISPLAY", display.DeviceString, NULL, NULL);

//        ScreenInfo screen;

//        char deviceName[32];
//        wcstombs(deviceName, display.DeviceName, 32);
//        screen.name = std::string(deviceName);

//        screen.resX = GetDeviceCaps(dc, HORZRES);
//        screen.resY = GetDeviceCaps(dc, VERTRES);

//        ret.push_back(screen);


//        DEVMODE settings;
//        ZeroMemory(&settings, sizeof(DEVMODE));
//        settings.dmSize = sizeof(DEVMODE);

//        EnumDisplaySettings(display.DeviceName, ENUM_CURRENT_SETTINGS, &settings);

//        int xpos = settings.dmPosition.x;
//        int ypos = settings.dmPosition.y;

//        DeleteDC(dc);
//    }

    return ret;
}

// Window Function
LONG WINAPI MainWndProc (
    HWND    hWnd,
    UINT    uMsg,
    WPARAM  wParam,
    LPARAM  lParam)
{
    LONG lRet = 1;
    PAINTSTRUCT ps;

    switch (uMsg) {

    case WM_PAINT:
        BeginPaint(hWnd, &ps);
        EndPaint(hWnd, &ps);
        break;
    default:
        lRet = DefWindowProc (hWnd, uMsg, wParam, lParam);
        break;
    }

    return lRet;
}

struct OpenGLContext::OpenGLContextInfo{
    HDC hdc;     // device OpenGLContext handle
    HGLRC hglrc; // OpenGL rendering OpenGLContext
    HWND hwnd;	 // window handle
    HINSTANCE hinstance; // application instance handle

    OpenGLContextInfo() : hdc(NULL), hglrc(NULL), hwnd(NULL){}
};

OpenGLContext::OpenGLContext(unsigned int _screenNum) : screenNum(_screenNum){

    std::vector<ScreenInfo> screenInfo = OpenGLContext::GetScreenInfo();
    screenResX = screenInfo[screenNum].resX;
    screenResY = screenInfo[screenNum].resY;

    // New object for OpenGLContext information
    contextInfo = new OpenGLContextInfo();

    // Create and register window class
    WNDCLASS wndclass;

    wndclass.style = CS_VREDRAW;
    wndclass.lpfnWndProc = (WNDPROC)MainWndProc;
    wndclass.cbClsExtra = 0;
    wndclass.cbWndExtra = 0;
    wndclass.hInstance = contextInfo->hinstance;
    wndclass.hIcon = LoadIcon(NULL, IDI_APPLICATION);
    wndclass.hCursor = LoadCursor(NULL, IDC_NO);
    wndclass.hbrBackground = NULL;
    wndclass.lpszMenuName = NULL;
    wndclass.lpszClassName = L"ProjectorGLClass";
    RegisterClass(&wndclass);

    // create the fullscreen window
    contextInfo->hwnd = CreateWindowEx(WS_EX_TOPMOST,L"ProjectorGLClass", L"Projector",
                                     WS_POPUP|WS_CLIPCHILDREN,
                                     screenInfo[screenNum].posX, screenInfo[screenNum].posY, screenResX, screenResY,
                                     NULL, NULL, contextInfo->hinstance, NULL);

//    contextInfo->hwnd = CreateWindow(L"ProjectorGLClass", L"Projector",
//                                     WS_POPUP|WS_EX_TOOLWINDOW,
//                                     screenInfo[screenNum].posX, screenInfo[screenNum].posY, screenResX, screenResY,
//                                     GetDesktopWindow(), NULL, contextInfo->hinstance, NULL);

    if(!contextInfo->hwnd)
        std::cerr << "Could not create window!" << std::endl;

    //CoTaskbarList list = CoTaskbarList();
    // Get window device OpenGLContext
    if ((contextInfo->hdc = GetDC(contextInfo->hwnd)) == NULL)
        std::cerr << "Could not get window device OpenGLContext!" << std::endl;

    // Select pixel format description
    PIXELFORMATDESCRIPTOR pfd, *ppfd;
    int pixelformat;

    ppfd = &pfd;

    ppfd->nSize = sizeof(PIXELFORMATDESCRIPTOR);
    ppfd->nVersion = 1;
    ppfd->dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    ppfd->dwLayerMask = PFD_MAIN_PLANE;
    ppfd->iPixelType = PFD_TYPE_COLORINDEX;
    ppfd->cColorBits = 8;
    ppfd->cDepthBits = 16;
    ppfd->cAccumBits = 0;
    ppfd->cStencilBits = 0;

    pixelformat = ChoosePixelFormat(contextInfo->hdc, ppfd);

    if ( (pixelformat = ChoosePixelFormat(contextInfo->hdc, ppfd)) == 0 )
        std::cerr << "Failed to choose pixel format!" << std::endl;

    if (SetPixelFormat(contextInfo->hdc, pixelformat, ppfd) == FALSE)
        std::cerr << "Failed to set pixel format!" << std::endl;

    // Create the OpenGL rendering OpenGLContext
    if ((contextInfo->hglrc = wglCreateContext(contextInfo->hdc)) == NULL)
        std::cerr << "Could not create OpenGL rendering OpenGLContext!" << std::endl;

    // Set swap interval
    //wglSwapIntervalEXT(1);

    // Make OpenGLContext current
    wglMakeCurrent(contextInfo->hdc, contextInfo->hglrc);

    // Show window
    ShowWindow(contextInfo->hwnd, SW_SHOW);
    UpdateWindow(contextInfo->hwnd);

    // Adjust gamma to one
    setGamma(1.0);
}

void OpenGLContext::setGamma(float gamma){
    // Adjust gamma

}

void OpenGLContext::makeContextCurrent(){
    wglMakeCurrent(contextInfo->hdc, contextInfo->hglrc);
}

void OpenGLContext::flush(){
    // Swap buffers
    SwapBuffers(contextInfo->hdc);
    // Synchronize CPU with vsync buffer swap
    //glFinish();
}

OpenGLContext::~OpenGLContext(){
    DestroyWindow(contextInfo->hwnd);
    wglMakeCurrent(contextInfo->hdc, NULL);	// release device OpenGLContext in use by rc
    wglDeleteContext(contextInfo->hglrc);	// delete rendering OpenGLContext

    delete contextInfo;
}


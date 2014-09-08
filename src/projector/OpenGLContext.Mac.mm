//
//  OpenGLContext.Mac.mm
//
//  Created by Jakob Wilm on 18/03/13.
//  Copyright (c) 2013 Jakob Wilm. All rights reserved.
//

#include "OpenGLContext.h"

#include <Cocoa/Cocoa.h>
#include <OpenGL/OpenGL.h>

struct OpenGLContext::OpenGLContextInfo{
    NSOpenGLContext *context;
    OpenGLContextInfo() : context(NULL){}
};

std::vector<ScreenInfo> OpenGLContext::GetScreenInfo(){
    std::vector<ScreenInfo> ret;

        NSArray *screens = [NSScreen screens];
        for (unsigned int i=0; i<[screens count]; i++) {
            NSRect frame = [[screens objectAtIndex:i] frame];
            ScreenInfo screenInfo;
            screenInfo.resX = frame.size.width;
            screenInfo.resY = frame.size.height;
            ret.push_back(screenInfo);
        }

    return ret;
}

OpenGLContext::OpenGLContext(unsigned int _screenNum){
    // Set instance var
    screenNum = _screenNum;
    std::vector<ScreenInfo> screens = GetScreenInfo();

    if(screenNum > screens.size())
        throw "Could not create OpenGLContext. Screen not available!";

    contextInfo = new OpenGLContextInfo();

    // Create a fullscreen OpenGL OpenGLContext on the specified screen
    NSAutoreleasePool *pool = [[NSAutoreleasePool alloc] init];

    // Instatiate NSApplication and connect to window server
    [NSApplication sharedApplication];

    // Chose application with dock icon (OS X Mavericks)
    [NSApp setActivationPolicy:NSApplicationActivationPolicyRegular];

    NSRect displayRect = [[[NSScreen screens] objectAtIndex:screenNum] frame];
    screenResX = displayRect.size.width;
    screenResY = displayRect.size.height;

    NSRect viewRect = NSMakeRect(0.0, 0.0, displayRect.size.width, displayRect.size.height);

    NSOpenGLPixelFormatAttribute attrs[] = {NSOpenGLPFADoubleBuffer,0};
    NSOpenGLPixelFormat* pixelFormat = [[NSOpenGLPixelFormat alloc] initWithAttributes:attrs];

    NSOpenGLView *openGLView = [[NSOpenGLView alloc] initWithFrame:viewRect pixelFormat:pixelFormat];

    NSDictionary *fullScreenOptions = [NSDictionary dictionaryWithObjectsAndKeys: [NSNumber numberWithBool:FALSE], @"NSFullScreenModeAllScreens", nil];
    [openGLView enterFullScreenMode:[[NSScreen screens] objectAtIndex:screenNum] withOptions:fullScreenOptions];

    contextInfo->context = [openGLView openGLContext];
    [contextInfo->context makeCurrentContext];

    // Set the swap interval to match vsync
    const GLint swapInterval = 1;
    [contextInfo->context setValues:&swapInterval forParameter:NSOpenGLCPSwapInterval];

    // Adjust gamma to one
    NSNumber *screenNumber = [[[[NSScreen screens] objectAtIndex:screenNum] deviceDescription] objectForKey:@"NSScreenNumber"];
    CGSetDisplayTransferByFormula([screenNumber intValue], 0, 1, 1, 0, 1, 1, 0, 1, 1);

//  // Disable cursor
//  [openGLView addCursorRect:viewRect cursor:[NSCursor crosshairCursor]];
    [pool drain];
}

void OpenGLContext::setGamma(float gamma){
    // Adjust gamma
    NSNumber *screenNumber = [[[[NSScreen screens] objectAtIndex:screenNum] deviceDescription] objectForKey:@"NSScreenNumber"];
    CGSetDisplayTransferByFormula([screenNumber intValue], 0, 1, gamma, 0, 1, gamma, 0, 1, gamma);
}

OpenGLContext::~OpenGLContext(){
    [[contextInfo->context view] exitFullScreenModeWithOptions:Nil];
    //[[_openGLContext view] dealloc];
}

void OpenGLContext::flush(){
    // Flush OpenGL commands
    [contextInfo->context flushBuffer];
    //glSwapAPPLE();
    //glFinishRenderAPPLE();
}

void OpenGLContext::makeContextCurrent(){
    [contextInfo->context makeCurrentContext];
}

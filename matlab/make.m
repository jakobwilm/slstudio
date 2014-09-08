%%
% Build SLStudio Matlab MEX functions
% Jakob Wilm, DTU, 2014

srcDir = '../src/';

%% Camera
srcDirCamera = [srcDir 'camera/'];
if ismac
    CXXFLAGS = {'-I/opt/local/lib/'};
    LDFLAGS = 'LDFLAGS = "\$LDFLAGS -ldc1394 -lueye_api"';
    DEFINES = {'-DWITH_CAMERAIIDC'};
    srcFilesCamera = {'Camera.cpp', 'CameraIIDC.cpp'};
elseif isunix
    CXXFLAGS = {'-I/usr/local/lib/', '-I/opt/XIMEA/include'};
    LDFLAGS = 'LDFLAGS = "\$LDFLAGS -ldc1394 -lm3api"';
    DEFINES = {'-DWITH_CAMERAIIDC', '-DWITH_CAMERAXIMEA'};
    srcFilesCamera = {'Camera.cpp', 'CameraIIDC.cpp', 'CameraXIMEA.cpp'};
elseif ispc
	CXXFLAGS = {'-IC:/Program Files/IDS/uEye/Develop/include/'};
    LDFLAGS = 'C:\Program Files\IDS\uEye\Develop\Lib\uEye_api_64.lib';
    DEFINES = {'-DWITH_CAMERAIDSIMAGING'};
    srcFilesCamera = {'Camera.cpp', 'CameraIDSImaging.cpp'};
end

srcFilesCamera = strcat(srcDirCamera, srcFilesCamera);
srcFilesCamera = ['CameraMex.cpp' srcFilesCamera];

mex('-v', ['-I' srcDirCamera], CXXFLAGS{:}, DEFINES{:}, srcFilesCamera{:}, LDFLAGS);

%% Projector
srcDirProjector = [srcDir 'projector/'];
srcFilesProjector = {'ProjectorOpenGL.cpp'};
if ismac
    srcFilesProjector = [srcFilesProjector 'OpenGLContext.Mac.cpp'];
    CXXFLAGS = 'CXXFLAGS = "\$CXXFLAGS -ObjC++"';
    LDFLAGS = 'LDFLAGS = "\$LDFLAGS -framework Cocoa -framework OpenGL"';
elseif isunix
    srcFilesProjector = [srcFilesProjector 'OpenGLContext.Unix.cpp'];
    CXXFLAGS = 'CXXFLAGS = "\$CXXFLAGS"';
    LDFLAGS = 'LDFLAGS = "\$LDFLAGS -lGL -lGLU -lX11 -lXxf86vm -lGLEW"';
elseif ispc
    srcFilesProjector = [srcFilesProjector 'OpenGLContext.Win.cpp'];
	CXXFLAGS = '-I.';
    LDFLAGS = '-lOpenGL32';
end

srcFilesProjector = strcat(srcDirProjector, srcFilesProjector);
srcFilesProjector = ['ProjectorMex.cpp' srcFilesProjector];

mex('-v', ['-I' srcDirProjector], srcFilesProjector{:}, CXXFLAGS, LDFLAGS);


%%
% Build SLStudio Matlab MEX functions
% Jakob Wilm, DTU, 2014

srcDir = '../src/';

%% Camera
srcDirCamera = [srcDir 'camera/'];
if ismac
    CXXFLAGS = {'-I/opt/local/lib/'};
    LDFLAGS = {'-ldc1394', '-lueye_api'};
    DEFINES = {'-DWITH_CAMERAIIDC'};
    srcFilesCamera = {'Camera.cpp', 'CameraIIDC.cpp'};
elseif isunix
    CXXFLAGS = {'-I/usr/local/lib/', '-I/opt/XIMEA/include'};
    LDFLAGS = {'-ldc1394', '-lm3api'};
    DEFINES = {'-DWITH_CAMERAIIDC', '-DWITH_CAMERAXIMEA'};
    srcFilesCamera = {'Camera.cpp', 'CameraIIDC.cpp', 'CameraXIMEA.cpp'};
elseif ispc
% 	CXXFLAGS = {'-IC:/Program Files/IDS/uEye/Develop/include/'};
%   LDFLAGS = 'C:\Program Files\IDS\uEye\Develop\Lib\uEye_api_64.lib';
%   DEFINES = {'-DWITH_CAMERAIDSIMAGING'};
    CXXFLAGS = {'-IC:/Program Files/Point Grey Research/FlyCapture2/include/'};
    LDFLAGS = { '-lFlyCapture2', '-LC:/Program Files/Point Grey Research/FlyCapture2/lib64'};
    DEFINES = {'-DWITH_CAMERAPOINTGREY', '-DWIN64'};
    srcFilesCamera = {'Camera.cpp', 'CameraPointGrey.cpp'};
end

srcFilesCamera = strcat(srcDirCamera, srcFilesCamera);
srcFilesCamera = ['CameraMex.cpp' srcFilesCamera];

mex('-v', ['-I' srcDirCamera], CXXFLAGS{:}, DEFINES{:}, srcFilesCamera{:}, LDFLAGS{:});

%% Projector
srcDirProjector = [srcDir 'projector/'];
srcFilesProjector = {'ProjectorOpenGL.cpp'};
if ismac
    srcFilesProjector = [srcFilesProjector 'OpenGLContext.Mac.cpp'];
    CXXFLAGS = {'CXXFLAGS = "\$CXXFLAGS -ObjC++"'};
    DEFINES = {};
    LDFLAGS = {'LDFLAGS = "\$LDFLAGS -framework Cocoa -framework OpenGL"'};
elseif isunix
    srcFilesProjector = [srcFilesProjector 'OpenGLContext.Unix.cpp'];
    CXXFLAGS = {'CXXFLAGS=$CXXFLAGS'};
    DEFINES = {};
    LDFLAGS = {'-lGL', '-lGLU', '-lX11', '-lXxf86vm', '-lGLEW'};
elseif ispc
    srcFilesProjector = [srcFilesProjector 'OpenGLContext.Win.cpp'];
	CXXFLAGS = {'-IC:\Program Files\glew-1.13.0\include'};
    DEFINES = {'-DUNICODE'};
    LDFLAGS = {'-lOpenGL32', '-LC:\Program Files\glew-1.13.0\lib\Release\x64', '-lglew32'};
end

srcFilesProjector = strcat(srcDirProjector, srcFilesProjector);
srcFilesProjector = ['ProjectorMex.cpp' srcFilesProjector];

mex('-v', ['-I' srcDirProjector], srcFilesProjector{:}, CXXFLAGS{:}, DEFINES{:}, LDFLAGS{:});


slstudio
========

##SLStudio -- Real Time Structured Light##

This software is designed to enable anyone to implement a custom 3D structured light scanner using a single camera and light projector. It is modular and has a focus on processing speed, enabling real-time structured light capture at 20 Hz and more. When using standard commercial projector and a webcam, the obtainable speed is lower due to the lack of hardware triggering.

When using the software in academic work, please consider citing the following publication.

Wilm et al., *SLStudio: Open-Source Framework for Real-Time Structured Light*, IPTA 2014 [IEEE Xplore](http://ieeexplore.ieee.org/xpl/articleDetails.jsp?reload=true&arnumber=7002001)

##Demo videos
[![SLStudio: Real-time 2x3 PSP](http://img.youtube.com/vi/tti4-9ADYLs/0.jpg)](https://www.youtube.com/watch?v=tti4-9ADYLs)
[![SLStudio: Calibration](http://img.youtube.com/vi/swszXuPxGZI/0.jpg)](https://www.youtube.com/watch?v=swszXuPxGZI)

##Compiling and installing
SLStudio is being developed with qmake (QtCreator). The /src/SLStudio.pro file is the project file which contains all information about the project and its dependencies needed for compiling. 
It is has a number of dependencies that you need to install before being able to compile the program on your machine:
* Qt 4.X (Qt 5 is not supported because most VTK packages link against Qt 4)
* OpenCV 2.8.X (3.0 could be used with relatively few modifications)
* PCL 1.7 (Point Cloud Library)
* VTK 5.8 including QVTK
* Boost 
* Eigen
* FLANN
* GLEW
* Depending on your camera: libdc1394, FlyCapture API, XIMEA m3api, IDS Imaging uEye API

The project has successfully been compiled on Ubuntu 14.04 -- 15.04, OS X 10.9 and Windows 7. The recommended OS is Ubuntu 15.04.

###Ubuntu 15.04
Ubuntu also has all of the dependencies available as packages (except camera libraries). Running the following line should have you (almost) set:
```
    sudo apt-get install libqt4-dev libpcl-dev libopencv-dev libglew-dev qtcreator
```
The greatest advantage of using Ubuntu is that you are able to render structured light patterns on a secondary X screen, which does not interfere with your main screen in which Unity and the SLStudio GUI run. Usually, this is an unusual use-case, as normally you are able to move the mouse or windows onto the second screen or use ALT-Tab. However, by setting up two X Screens in xorg.conf with a gap in between them, you can make Unity completely ignore the second screen so it is only SLStudio that draws onto it. This also depends on your graphics driver supporting multiple X screens (work with current proprietary nVidia and AMD drivers).

###OS X 10.10
On OS X, the dependencies are available through MacPorts. You will not get completely independent screens, so your measurements may be corrupted by GUI activity. Otherwise the program runs well.

###Windows 7/8/10
On Windows machines, OpenCV and PCL are available as downloads, but since everything needs to be ABI compatible, it is probably best to compile the dependencies from source with the same compiler (e.g. MSVC) which is rather time-consuming but possible. It is some time since we have last compiled on Windows, but all of the code should be compatible. 

###Camera APIs
Because high performance operation of cameras often requires vendor- or device-specific features or setup, we choose not to use an intermediate layer such as OpenCV to communicate with the camera. Currently, wrappers for a number of camera APIs is provided, and we invite anyone who implements others to contribute them. Be aware that we have only tested on a limited number of cameras, and some of the configurations performed may only apply to our specific models (e.g. images are assumed to be in grayscale). In any case, you may need to modify some of the camera code to get this to work with hardware triggering (required for real time performance).

###Matlab wrappers
The project also contains Matlab mex-wrappers for the OpenGL projector and Camera. This makes it possible to e.g. determine the gamma response of your camera-projector setup, or other debugging tasks. The mex-wrappers are compiled from Matlab by running matlab/make.m. Tested on Ubuntu only.

##Running

###Projector source
SLStudio can run in different projector modes. For projection you can choose OpenGL projection, which renders the structured light patterns on "screen 2". You would then use an HDMI connection to the projector. Specifically for LightCrafter and LightCrafter4500, the software can communicate over USB with these projectors, which can preload the patterns. However, this does currently not work under robustly, and not in the calibration phase. 

###Camera Triggering
In the preference pane, software and hardware triggering can be selected. The calibration procedure allways uses software triggering. For real-time structured light, you need hardware triggering from the projector to the camera by means of a trigger signal cable. Few projectors provide the trigger output signal, but with some effort you may be able to source one from the HDMI vsync pin. LightCrafter4500 has trigger output pins and is the recommended device. You will have to produce a trigger cable with the specific headers of your projector and camera. 
With commercial projectors you can usually project 60 patterns per second at most (LC4500 allows for 120 8-bit patterns per second). Usually the camera requires a pause between two consecutive exposures, and you will need to waste one refresh period. With LC4500 we can project 60 grayscale patterns per second resulting in up to 20 point clouds per second.

###Calibration
Calibration of your hardware setup determines the internal camera and projector parameters, and the relative position of the projector relative to the camera. The calibration procedure only is seen in the video above, and requires a flat board with a grey-white checkerboard printed onto it. Good calibration requires a large number of calibration positions covering all of the sensor field of view and with some foreshortening (angle of the calibration board relative to view-axis).

###Codec
SLStudio serves as a structured light platform. A number of encoding strategies has been implemented, and these differ greatly in the number of patterns, accuracy and robustness to e.g. shiny or semitransparent surfaces. For many applications, 2x3 phase shifting provides good results. It uses 6 patterns, so you will reach 10Hz point cloud update frequency with the appropriate hardware. 

###Recommended settings
Please note that some parts of this software are still experimental, while others have been matured. For reliable point cloud capture we recommend the following choices:
* Ubuntu 15.04
* Newer dedicated nVidia graphics card
* Ximea or Point Grey camera
* Calibration in 20 positions
* Calibration board with 10 x 10 saddle points
* Software triggering
* 2x3 Phase Shifting

For real-time (10Hz +) performance we recommend the following:
* LightCrafter4500 configured for 120Hz HDMI gray-scale projection (e.g. configure using /tools/lc4500startup)
* Ximea MQ013RG-E2
* Hardware triggering

##Support
While we are interested in providing our software to a rich audience and make real-time structured light available to many user groups, it usually requires a fair amount of customization and knowledge to build these systems. With the current state of this project and the challenges in hardware, you probably do need some C++ programming experience to make it work with your specific setup, unless it consists of the exact components listed above. We can answer some questions pertaining to software and hardware, but cannot provide low-level support. We are though very interested in incorporating improvements and bug fixes, so if you have such, please contact us or make a pull request!

Regards, Jakob, Technical University of Denmark



%CAMERA Matlab wrapper class for the Camera C++ class.
% Gives low level access to custom industrial cameras by wrapping their
% API. Currently libdc1394 IIDC and IDS Imaging uEye are implemented.
%
% Jakob Wilm, DTU 2013

classdef Camera < handle
    properties (SetAccess = private, Hidden = true)
        objectHandle; % Handle to the underlying C++ class instance
    end
    methods (Static = true)
        function varargout = GetInterfaceCameraList
            [varargout{1:nargout}] = CameraMex('GetInterfaceCameraList');
        end
    end
    methods
        % Constructor - Create a new C++ class instance 
        function this = Camera(interfaceNum, cameraNum)
            this.objectHandle = CameraMex('NewCamera', interfaceNum, cameraNum);
        end
        
        % Destructor - Destroy the C++ class instance
        function delete(this)
            CameraMex('delete', this.objectHandle);
        end

         % startCapture
        function startCapture(this)
            CameraMex('startCapture', this.objectHandle);
        end
        
         % stopCapture
        function stopCapture(this)
            CameraMex('stopCapture', this.objectHandle);
        end
        
        % getFrame
        function varargout = getFrame(this)
            frame = CameraMex('getFrame', this.objectHandle);
            [varargout{1:nargout}] = transpose(frame);
        end
        
    end
end

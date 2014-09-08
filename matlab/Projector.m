%PROJECTOR Matlab wrapper class for the Projector C++ class.
% Creates a fullscreen OpenGL context for the screen specified for fast
% texture or pattern projection.
%
% Jakob Wilm, DTU 2013

classdef Projector < handle
    properties (SetAccess = private, Hidden = true)
        objectHandle; % Handle to the underlying C++ class instance
    end
    methods (Static = true)
        function varargout = GetScreenInfo
            [varargout{1:nargout}] = ProjectorMex('GetScreenInfo');
        end
    end
    methods
        % Constructor - Create a new C++ class instance 
        function this = Projector(screenNum)
            this.objectHandle = ProjectorMex('new', screenNum);
        end
        
        % Destructor - Destroy the C++ class instance
        function delete(this)
            ProjectorMex('delete', this.objectHandle);
        end

        % displayTexture
        function varargout = displayTexture(this, texture)
            height = size(texture, 1);
            width = size(texture, 2);
            % swivel data to match RMO OpenGL format
            texture = permute(texture, [3 2 1]);
            [varargout{1:nargout}] = ProjectorMex('displayTexture', this.objectHandle, texture, width, height);
        end
   
        % displayWhite
        function varargout = displayWhite(this)
            [varargout{1:nargout}] = ProjectorMex('displayWhite', this.objectHandle);
        end
        
        % displayBlack
        function varargout = displayBlack(this)
            [varargout{1:nargout}] = ProjectorMex('displayBlack', this.objectHandle);
        end

    end
end

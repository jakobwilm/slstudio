function calib = readSLCalib(fileName)
%READSLCALIB reads calibration parameters for one camera and one
%projector in the SLStudio calibration file format.
%
% Input arguments:
%           fileName: string with the file name. 
%
% Output arguments:
%           calib: struct containing all or some of the following fields:
%               .Kc: 3x3 camera intrinsic matrix.
%               .Rc: 3x3 camera extrinsic rotation matrix.
%               .Tc: 3x1 camera extrinsic translation vector.
%               .kc: 5x1 camera lens distortion vector.
%               .Kp: 3x3 projector intrinsic matrix.
%               .Rp: 3x3 projector extrinsic rotation matrix.
%               .Tp: 3x1 projector extrinsic translation vector.
%               .kp: 5x1 projector lens distortion vector.
%
% Note that the lens distortion vectors, kc and kp follow the general
% Bouget toolbox convention that is also used in OpenCV.
%
% DTU 2013, Jakob Wilm


%calib = struct('Kc', zeros(3), 'Rc', eye(3), 'Tc', zeros(3,1), 'kc', zeros(5,1), ...
%               'Kp', zeros(3), 'Rp', eye(3), 'Tp', zeros(3,1), 'kp', zeros(5,1));
params = {'Kc', 'Rc', 'Tc', 'kc', 'Kp', 'Rp', 'Tp', 'kp'};

fid = fopen(fileName);

calib = [];

while ~feof(fid) %while not at end of file
    line = fgets(fid); %fgets() is faster than fgetl().
    idx = strncmp(line, params, 2); 
    if any(idx) %see if we found a header
        if params{idx}(1) == 'k'
            calib.(params{idx}) = fscanf(fid, '%f', [5,1]);
        elseif params{idx}(1) == 'T'
            calib.(params{idx}) = fscanf(fid, '%f', [3,1]);
        else
            calib.(params{idx}) = transpose(fscanf(fid, '%f %f %f', [3,3]));
        end
    end
end

fclose(fid);

end


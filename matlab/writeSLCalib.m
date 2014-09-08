function flag = writeSLCalib(fileName, calib)
%WRITESLCALIB writes calibration parameters for one camera and one
%projector in the SLStudio calibration file format.
%
% Input arguments:
%           fileName: string with the file name. If no extension is
%           provided, the default *.slcalib is used.
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
% Output arguments:
%           flag: boolean value indicating if write was successful. 
%
% Note that the lens distortion vectors, kc and kp follow the general
% Bouget toolbox convention that is also used in OpenCV.
%
% DTU 2013, Jakob Wilm

fid = fopen(fileName, 'w');
if fid == -1
    error(['Could not write calibration file ' fileName]);
end

fprintf(fid, '#V1.0 SLStudio calibration file\n\n');

params = fieldnames(calib);

for p=1:length(params)
    fprintf(fid, '%s\n', params{p});
    param = calib.(params{p});
    for i=1:size(param,1)
        for j=1:size(param,2)
            fprintf(fid, '%f\t', param(i,j));
        end
        fprintf(fid, '\n\n');
    end
end

fclose(fid);
flag = true;
end


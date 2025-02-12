function [R] = eAngles2rotM(yaw, pitch, roll)
% [R] = eAngles2rotM(yaw, pitch, roll)
% Computes the rotation matrix R given the Euler angles (yaw, pitch, roll). 
% Inputs:
%	yaw: angle of rotation around the z axis
%	pitch: angle of rotation around the y axis
%	roll: angle of rotation around the x axis
% Outputs:
%	R: rotation matrix

r_yaw = [cosd(yaw), sind(yaw), 0;
        -sind(yaw), cosd(yaw), 0;
        0, 0, 1];
    
r_pitch = [cosd(pitch), 0, -sind(pitch);
        0, 1, 0;
        sind(pitch), 0, cosd(pitch)];
    
r_roll = [1, 0, 0;
        0, cosd(roll), sind(roll);
        0, -sind(roll), cosd(roll)];

R = transpose(r_roll * r_pitch *r_yaw);

end





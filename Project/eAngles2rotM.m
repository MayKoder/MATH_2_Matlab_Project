function [R] = eAngles2rotM(yaw, pitch, roll)
% [R] = eAngles2rotM(yaw, pitch, roll)
% Computes the rotation matrix R given the Euler angles (yaw, pitch, roll). 
% Inputs:
%	yaw: angle of rotation around the z axis
%	pitch: angle of rotation around the y axis
%	roll: angle of rotation around the x axis
% Outputs:
%	R: rotation matrix

r_yaw = [cos(yaw), sin(yaw), 0;
        -sin(yaw), cos(yaw), 0;
        0, 0, 1];
    
r_pitch = [cos(pitch), 0, -sin(pitch);
        0, 1, 0;
        sin(pitch), 0, cos(pitch)];
    
r_roll = [1, 0, 0;
        0, cos(roll), sin(roll);
        0, -sin(roll), cos(roll)];

R = transpose(r_roll * r_pitch *r_yaw);

end





function [yaw, pitch, roll] = rotM2eAngles(R)
% [yaw, pitch, roll] = rotM2eAngles(R)
% Computes the Euler angles (yaw, pitch, roll) given an input rotation matrix R.
% Inputs:
%	R: rotation matrix
% Outputs:
%	yaw: angle of rotation around the z axis
%	pitch: angle of rotation around the y axis
%	roll: angle of rotation around the x axis

local_R = R; 

if abs(R(3, 1)) == 1 
    yaw = 0;
    if R(3, 1) == 1   
        pitch = 90;           
    else     
        pitch = 270;
    end    
    
    roll = atan2d((local_R(3, 2) / cos(pitch)), ((local_R(3, 3) / cos(pitch))));
    
    
else
    
    pitch = asind(-R(3, 1));

    yaw = atan2d((local_R(2, 1) / cos(pitch)), ((local_R(1, 1) / cos(pitch))));
    roll = atan2d((local_R(3, 2) / cos(pitch)), ((local_R(3, 3) / cos(pitch)))); 
    
end
end





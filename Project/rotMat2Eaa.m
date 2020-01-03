function [a,u] = rotMat2Eaa(R)
% [a,u] = rotMat2Eaa(R)
% Computes the angle and principal axis of rotation given a rotation matrix R. 
% Inputs:
%	R: rotation matrix
% Outputs:
%	a: angle of rotation
%	u: axis of rotation 


%Return angle in rads
preAngle = (trace(R)-1)/2;

%Make sure angle is between -1 and 1
if preAngle >= -1 && preAngle <= 1
    a=acosd((trace(R)-1)/2);
else
    a = 0;
end

%Calculate u matrix
test = (R-R.');

if test == zeros(length(R), length(R))
    u = [1; 1; 1] / sqrt(3);
else
    mat2=(test/(2*sin(a)));
    
    %Return u axis
    u=[-mat2(2,3);mat2(1,3);-mat2(1,2)];
end

end


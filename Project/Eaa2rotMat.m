function [R] = Eaa2rotMat(a,u)
% [R] = Eaa2rotMat(a,u)
% Computes the rotation matrix R given an angle and axis of rotation. 
% Inputs:
%	a: angle of rotation
%	u: axis of rotation 
% Outputs:
%	R: generated rotation matrix

%Normalize matrix
u = u/ norm(u);

len = length(u);
s_cos =cosd(a);
s_sen =sind(a);

I = eye(len);
uuT = u * transpose(u);

Ux = [0,-u(3),u(2);
    u(3),0,-u(1);
    -u(2),u(1),0];

R = (I*s_cos)+((1-s_cos)*uuT) +(Ux*s_sen);

end


function rotation_matrix = Quat2rotMat(q)
%QUAT2ROTMAT This function returns the conversion of a quaternion to a
%rotation matrix.
%Input: quaternion
%Output: rotation matrix

q = q/norm(q);

rotation_matrix = [q(1)^2 + q(2)^2 - q(3)^2 - q(4)^2, 2*q(2)*q(3) - 2*q(1)*q(4), 2*q(2)*q(4) + 2*q(1)*q(3);
                   2*q(2)*q(3) + 2*q(1)*q(4), q(1)^2 - q(2)^2 + q(3)^2 - q(4)^2, 2*q(3)*q(4) - 2*q(1)*q(2);
                   2*q(2)*q(4) - 2*q(1)*q(3), 2*q(3)*q(4) + 2*q(1)*q(2), q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2];

end
function rotation_vector = rotM2rotVec(rotation_matrix)
%ROTM2ROTVEC  This function returns the rotation vector given a rotation
%matrix.
%   Input: rotation matrix
%   Output: rotation vector

rotation_vector(2) = asind(rotation_matrix(3, 1));
rotation_vector(1) = atan2d((rotation_matrix(3, 2) / cosd(rotation_vector(2))), (rotation_matrix(3, 3) / cosd(rotation_vector(2))));
rotation_vector(3) = atan2d((rotation_matrix(2, 1) / cosd(rotation_vector(2))), (rotation_matrix(1, 1) / cosd(rotation_vector(2))));

end


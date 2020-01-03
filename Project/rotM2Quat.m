function quaternion = rotM2Quat(rotation_matrix)
%ROTM2QUAT This function returns the quaternion given a rotation matrix.
%   Input: rotation matrix
%   Output: quaternion, with dimensions [4, 1]

quaternion(1, 1) = sqrt(1 + rotation_matrix(1, 1) + rotation_matrix(2, 2) + rotation_matrix(3, 3)) / 2;
quaternion(2, 1) = (rotation_matrix(3, 2) - rotation_matrix(2, 3)) / (4 * quaternion(1, 1));
quaternion(3, 1) = (rotation_matrix(1, 3) - rotation_matrix(3, 1)) / (4 * quaternion(1, 1));
quaternion(4, 1) = (rotation_matrix(2, 1) - rotation_matrix(1, 2)) / (4 * quaternion(1, 1));

end

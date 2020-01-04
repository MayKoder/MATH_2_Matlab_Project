function rotation_matrix = rotVec2rotMat(rot_vector)
%ROTVEC2ROTMAT This functions returns the rotation matrix of a given 
%rotation vector.

%   Input: rotation vector
%   Output: rotation matrix

normalized_vector = norm(rot_vector);
rot_vector = rot_vector / normalized_vector;

rot_mat = [0 -rot_vector(3) rot_vector(2);
             rot_vector(3) 0 -rot_vector(1);
             -rot_vector(2) rot_vector(1) 0];

rotation_matrix = eye(3) * cosd(normalized_vector) + (1 - cosd(normalized_vector)) * (rot_vector * rot_vector') + sind(normalized_vector) * rot_mat ;

end


function [tf_april_to_camera, tf_ee_to_base] = read_poses(file_name)
data = importdata(file_name, ' ');
num_poses = size(data, 1) / 8;
for i = 1:1:num_poses
    tf_april_to_camera{i} = data(8*(i-1)+1 : 8*(i-1) + 4, :);
    %tf_april_to_camera{i}(1:3,1:3) =  [1 0 0; 0 1 0; 0 0 1] * tf_april_to_camera{i}(1:3,1:3);
     %tf_april_to_camera{i}(1:3,1:3) = [0,0,1;-1,0,0;0,-1,0]* tf_april_to_camera{i}(1:3,1:3);
%     tf_april_to_camera{i} = inv([0,0,1, 0;....
%                                         -1,0,0, 0;0,-1,0, 0; 0,0,0,1]) * tf_april_to_camera{i};
     tf_ee_to_base{i} = data(8*(i-1)+5 : 8*i, :);
end
end
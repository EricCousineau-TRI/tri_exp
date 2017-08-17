% R_park = [0.259853173940471  -0.965537524450837   0.014615637872150  
%    0.965223130815976   0.260158061712116   0.025731122477099  
%   -0.028646740317385   0.007421037901104   0.999562050332874];

R_park = X_park(1:3,1:3);

% Compute the translated apriltag positions. 
num_poses = length(tf_april_to_camera);
pos_aprils = zeros(3, num_poses);
for i = 1:1:num_poses
Re{i} = tf_ee_to_base{i}(1:3,1:3);
Ra{i} = tf_april_to_camera{i}(1:3,1:3);
te{i} = tf_ee_to_base{i}(1:3, 4);
ta{i} = tf_april_to_camera{i}(1:3,4);
end

%t_april_to_world = [1.61; -0.2; 0.5];
t_april_to_world = [0.65; 0; -0.1];
R = R_park;
outer_loop = 0;
max_iter_outer_loop = 100;
while (outer_loop < max_iter_outer_loop)
% Fix rotation matrix and find best translation. 
iters = 0;
max_iters = 100;
while (iters < max_iters)
    % Fix t_april_to_world, find best position offset of camera w.r.t ee. 
    t_c_to_ee = zeros(3, num_poses);    
    for i = 1:1:num_poses
        t_c_to_ee(:, i) = inv(Re{i}) * (t_april_to_world - Re{i} * R * ta{i} - te{i});
    end
    t = mean(t_c_to_ee, 2);
    % fix t, find best t_april_to_world
    t_april_to_world_all = zeros(3, num_poses);
    for i = 1:1:num_poses
        t_april_to_world_all(:, i) = Re{i} * R * ta{i} + te{i} + Re{i} * t;
    end
    t_april_to_world = mean(t_april_to_world_all, 2);
    iters = iters + 1;
end
% Now fix translation. Find best rotation. 
X = zeros(3, num_poses);
Y = zeros(3, num_poses);
for i = 1:1:num_poses
    X(:,i) = ta{i};
    Y(:,i) = inv(Re{i}) * (t_april_to_world - te{i}) - t;
end
% Use SVD trick.
X = bsxfun(@minus, X, mean(X, 2));
Y = bsxfun(@minus, Y, mean(Y, 2));
H = X * Y';
[U, S, V] = svd(H);
R = V * U';
R, t
outer_loop = outer_loop + 1;
end



X_new = eye(4,4);
X_new(1:3, 1:3) = R;
X_new(1:3, 4) = t;
X_new




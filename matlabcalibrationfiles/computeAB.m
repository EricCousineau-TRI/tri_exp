function [A, B] = computeAB(tf_april_to_camera, tf_ee_to_base)
num_poses = length(tf_april_to_camera);
A = [];
B = [];
%num_poses = 2;
for i = 2:1:num_poses
%     A =[ A inv(tf_ee_to_base{i-1}) * tf_ee_to_base{i}];
%     B =[ B tf_april_to_camera{i} * inv(tf_april_to_camera{i-1}) ];
%      A =[ A inv(tf_ee_to_base{i}) * tf_ee_to_base{1}];
%     B =[ B tf_april_to_camera{i} * inv(tf_april_to_camera{1}) ];
%A =[ A, inv(tf_ee_to_base{i-1}) * (tf_ee_to_base{i})  ];
%B =[ B, tf_april_to_camera{i-1} * inv(tf_april_to_camera{i}) ];
A =[ A (tf_ee_to_base{1}) \ (tf_ee_to_base{i})  ];
B =[ B tf_april_to_camera{1} / (tf_april_to_camera{i}) ];

end
end
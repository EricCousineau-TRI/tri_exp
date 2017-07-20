[tf_april_to_camera, tf_ee_to_base] = read_poses('output.txt');
[A, B] = computeAB(tf_april_to_camera, tf_ee_to_base);
X_park = park(A,B)
X_l =liang(A,B)
%sX_tsai = tsai(A,B)
tf_ee_to_base{4} * X_park * tf_april_to_camera{4}
tf_ee_to_base{4} * X_l * tf_april_to_camera{4}
tf_ee_to_base{9} * X_park * tf_april_to_camera{9}
tf_ee_to_base{9} * X_l * tf_april_to_camera{9}
function mat = homogenous_mat(pos, euler_angles)
% function mat = homogenous_mat(pos, eulerZYX)
    roll=euler_angles(1);
    pitch=euler_angles(2);
    yaw=euler_angles(3);
    x=pos(1);
    y=pos(2);
    z=pos(3);
    mat = zeros(4,4);
    
    mat=[Rzyx(roll,pitch,yaw) [x;y;z]; [0,0,0,1]];

end

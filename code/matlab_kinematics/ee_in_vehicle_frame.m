function vec = ee_in_vehicle_frame(q,d,a,alpha)
    
    g=eye(4);
    for i=1:6
        g=g*dh_homogenous(q(i),d(i),a(i),alpha(i));
    end
    
    Rtemp=Rzyx(0,-pi/2,pi/2);
    tool_hom_mat=[Rtemp,[0;0;0];[0,0,0,1]];
    g=g*tool_hom_mat;
    vec=zeros(3,1);
    vec=g(1:3,4);

end



% % get the homogenous matrices between each frame and frame zero
% function aCell = get_homogenous_matrices(q, d,a,alpha)
%     
%     %global a d alpha;
%     coder.extrinsic('cell');
%     aCell=cell(6,1);
%     for i=1:6,
%         aCell{i} = dh_homogenous(q(i),d(i),a(i),alpha(i));
%     end
%     Rtemp=Rzyx(0,-pi/2,pi/2);
%     tool_hom_mat=[Rtemp,[0;0;0];[0,0,0,1]];
%     aCell{i} = aCell{i}*tool_hom_mat;
% end


% homogenous transformation matrix - DH convention
% mat=Rot_z(theta)Trans_z(d)Trans_x(a)Rot_x(alpha)
function mat=dh_homogenous(theta,d,a,alpha)
    mat=eye(4);
    mat=[cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
         sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0, sin(alpha), cos(alpha), d;
         0,0,0,1
    ];

end
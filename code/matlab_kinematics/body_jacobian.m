% ------------------------------------------
%
% body jacobian mapping quasi velocities to 
% end effector twist
%
%
% ------------------------------------------

function J = body_jacobian(q)
    h_mat=cell(6,1);
    h_mat=get_homogenous_matrices(q);
    ad_gbe_inv=Ad_gbe_inv(h_mat);
    jsmg=Jsmg(h_mat);
    J=[ad_gbe_inv, ad_gbe_inv*jsmg];
end


% get the inverse of the adjoint map from frame b to frame e
function mat=Ad_gbe_inv(h_mat)
    g0e=h_mat{1};
    for i=2:6
        g0e=g0e*h_mat{i};
    end
    p=g0e(1:3,4);
    R=g0e(1:3,1:3);
    rt=transpose(R);
    rp=-rt*skew(p);
    mat(1:3,1:3)=rt;
    mat(1:3,4:6)=rp;
    mat(4:6,1:3)=zeros(3,3);
    mat(4:6,4:6)=rt;    
end


% the manipulator jacobian for spatial coordinates
% mapping the twists from qd to vd
function mat=Jsmg(h_mat)
    pos_vec = cell(6,1);
    R_mat = cell(6,1);
    
    adgbi=cell(6,1);
    g0i=cell(6,1);
    g0i{1}= eye(4);%h_mat{1};
    R_mat{1}=eye(3); %hom2R(h_mat{1});
    pos_vec{1}=zeros(3,1); %hom2pos(h_mat{1});
    for i=2:6
       g0i{i}=g0i{i-1}*h_mat{i-1};
       pos_vec{i}=hom2pos(g0i{i});
       R_mat{i}=hom2R(g0i{i});
    end
        
        
    for i=1:6
        adgbi{i}=pose2adgb(pos_vec{i}, R_mat{i});
    end
    
    mat =zeros(6,6);
    for i=1:6
       mat(1:6,i)=adgbi{i}(1:6,6); 
    end
    
end

% construct the adjoint matrix from the homogenous matrix
function mat = homogenous2ad(h_mat)
    R=h_mat(1:3,1:3);
    p=h_mat(1:3,4);
    mat=[R, skew(p)*R; eye(3)*0, R];
end


function mat = pose2adgb(pos,R)
    mat = zeros(6,6);
    mat = [R, skew(pos)*R; zeros(3,3), R];
end

% make the skew symmetric matrix out of a vector
function mat = skew(vec)
    mat=[0, -vec(3), vec(2);
        vec(3), 0 , -vec(1);
        -vec(2), vec(1), 0];
end

% get the homogenous matrices between each frame and frame zero
function aCell = get_homogenous_matrices(q)
    global a d alpha;
    aCell=cell(6,1);
    for i=1:6,
        aCell{i} = dh_homogenous(q(i),d(i),a(i),alpha(i));
    end
    Rtemp=Rzyx(0,-pi/2,pi/2);
    tool_hom_mat=[Rtemp,[0;0;0];[0,0,0,1]];
    aCell{i} = aCell{i}*tool_hom_mat;
end


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


function vec = hom2pos(hmat)
    vec = hmat(1:3,4);
end

function mat = hom2R(hmat)
    mat  = hmat(1:3,1:3);
end






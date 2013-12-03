%-----------------------------------
%
%  this file holds the parameters 
%  defining the kinematic structure
%  in Denavit-Hartenbarg terms
%
%-----------------------------------

clear all;clc;

%% The denavit-hartenberg defines each frame as a homognous transformation
% definded by:
%               Ai=Rot_z(theta)Trans_z(d)Trans_x(a)Rot_x(alpha)
%


d2r=pi/180;
r2d=180/pi;

global a;%1 a2 a3 a4 a5 a6;
global d;%1 d2 d3 d4 d5 d6;
global alpha;%1 alpha2 alpha3 alpha4 alpha5 alpha6;
a(1) = 0.2;
a(2) = 1;
a(3) = 0.6;
a(4) = 0.4;
a(5) = 0;
a(6) = 0;

d(1) = 0;
d(2) = 0;
d(3) = 0;
d(4) = 0;
d(5) = 0;
d(6) = 0.4;

alpha(1) = pi/2;
alpha(2) = 0;
alpha(3) = 0;
alpha(4) = -pi/2;
alpha(5) = -pi/2;
alpha(6) = 0;


%% joint limits - global parameters 

global qmin;
global qmax;

qmin(1)=-70*(pi/180);
qmax(1)=70*(pi/180);

qmin(2)=-70*(pi/180);
qmax(2)=70*(pi/180);

qmin(3)=-70*(pi/180);
qmax(3)=70*(pi/180);

qmin(4)=-70*(pi/180);
qmax(4)=70*(pi/180);

qmin(5)=90*(pi/180) - 70*(pi/180);
qmax(5)=70*(pi/180) + 90*(pi/180);

qmin(6)=-160*(pi/180);
qmax(6)=160*(pi/180);




%% set robot configurations
global six_link;

for i =1:6
    l(i) = Link([0, d(i), a(i), alpha(i)]);
end
% offset the angle of the 5th joint
%l(5).offset=-pi/2;

l(1).m=1;
l(2).m=3;
l(3).m=2;
l(4).m=1;
l(5).m=0.4;
l(6).m=0.8;
l(1).r = [-a(1)/2,0,0];
l(2).r=[-a(2)/2,0,0];
l(3).r=[-a(3)/2,0,0];
l(4).r=[-a(4)/2,0,0];
l(5).r=[0,0,0];
l(6).r=[0,0,-d(6)/2];
l(1).I=eye(3)*0.2;
l(2).I=eye(3)*0.5;
l(3).I=eye(3)*0.4;
l(4).I=eye(3)*0.2;
l(5).I=eye(3)*0.1;
l(6).I=eye(3)*0.1;

l(1).B=1;
l(1).G=1;
l(1).Jm=1;
l(2).B=1;
l(2).G=1;
l(2).Jm=1;
l(3).Jm=1;
l(4).Jm=1;
l(5).Jm=1;
l(6).Jm=1;

six_link = SerialLink(l,'name','six link');

clear i;
initial_q=[0,65*d2r,-(65+90)*d2r,90*d2r,-90*d2r,0];             % initial joint links
initial_eta=[0,0,0,0,0,0];                                  % initial vehicle pose                                             
hom_mat=homogenous_mat(initial_eta(1:3), initial_eta(4:6));

six_link.base=hom_mat;
tool_hom_mat=zeros(4,4);
Rtemp=Rzyx(0,-pi/2,pi/2);
tool_hom_mat=[Rtemp,[0;0;0];[0,0,0,1]];
six_link.tool=tool_hom_mat;

ee_pose_hom = six_link.fkine(initial_q);
initial_ee_eta = zeros(1,6);
initial_ee_eta(1:3) = ee_pose_hom(1:3,4);
R=ee_pose_hom(1:3,1:3);
[phi,theta,psi]=R2euler(R);
initial_ee_eta(4:6)=[phi,theta,psi];



%% ROV parameters
global m_rov;
r_g = [-1,0,0.5];       % vector from Frame b to center of gravity
mass=1000;
m_rov=[mass*eye(3) -mass*Smtrx(r_g) ; mass*Smtrx(r_g) mass*eye(3)];


%% Control parameters

K_p=diag([1,1,1,1,1,1,   1,1,1,1,1,1]);
K_i=diag([1,1,1,1,1,1,   1,1,1,1,1,1])*0.5 * 0;
K_pos_correction = diag([0,0.5,0.5 ,0,0.4,0.4])*10 ;


%% environment

current_s = [0,1,0,0,0,0, 0,0,0,0,0,0] * 0 ;


%% set path parameters

zeta_command = [0,0.,0,0,0.3,0,0.3,0,0.6,0,0.4,0];
v_ee_c = [0.1,0,0,0,0,0];
inner_motion_vec = [0,0,0,0,0,1,0,0,0,0,0,0];





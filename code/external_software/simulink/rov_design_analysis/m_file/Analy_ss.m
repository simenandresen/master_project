% Select operating points -> uo, wo for analysis and control
clc;
sys=ss(Ao,Bo,Co,Do);

% ss matrices of 1st order Thruster model

%Thruster dynamic
GT=tf([0.97],[0.02 1]);
disp('Linear Thruster Model (4 x 4 matrices')
G44=[GT 0 0 0; 0 GT 0 0; 0 0 GT 0; 0 0 0 GT]
G44_thruster=G44*pinv(T);

%sys matrix- thruster
sys_thruster=canon(G44_thruster,'modal')
[AT2,BT2,CT2,DT2]=ssdata(sys_thruster);

% State space - rov + thruster 
disp('Linear Thruster + Linear ROV model (in state-space)')
sys_tot=sys*sys_thruster;
[At,Bt,Ct,Dt]=ssdata(sys_tot)

% TF matrix -rov + thruster
disp('Linear Thruster + Linear ROV model (in transfer function matrix)')
[num1 den]=ss2tf(At,Bt,Ct,Dt,1);
[num2 den]=ss2tf(At,Bt,Ct,Dt,2);
[num3 den]=ss2tf(At,Bt,Ct,Dt,3);
[num4 den]=ss2tf(At,Bt,Ct,Dt,4);

G11=tf(num1(1,:),den);
G21=tf(num1(2,:),den);
G31=tf(num1(3,:),den);
G41=tf(num1(4,:),den);
G51=tf(num1(5,:),den);
G61=tf(num1(6,:),den);

G12=tf(num2(1,:),den);
G22=tf(num2(2,:),den);
G32=tf(num2(3,:),den);
G42=tf(num2(4,:),den);
G52=tf(num2(5,:),den);
G62=tf(num2(6,:),den);

G13=tf(num3(1,:),den);
G23=tf(num3(2,:),den);
G33=tf(num3(3,:),den);
G43=tf(num3(4,:),den);
G53=tf(num3(5,:),den);
G63=tf(num3(6,:),den);


G14=tf(num4(1,:),den);
G24=tf(num4(2,:),den);
G34=tf(num4(3,:),den);
G44=tf(num4(4,:),den);
G54=tf(num4(5,:),den);
G64=tf(num4(6,:),den);

G46_rov=[G11 G12 G13 G14;
   G21 G22 G23 G24 ;
   G31 G32 G33 G34 ;
   G41 G42 G43 G44 ;
   G51 G52 G53 G54 ;
   G61 G62 G63 G64 ]

% tf matrix - rov + thruster 
G_tot=G46_rov*G44_thruster;










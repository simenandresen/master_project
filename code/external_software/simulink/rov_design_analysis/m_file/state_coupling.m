
clc
cd c:\Matlab7\toolbox\rov_design_analysis\m_file
load rrcrovf 

uo(1)=0.01;
vo(1)=0.01;
wo(1)=0.01;
po(1)=0.01;
qo(1)=0.01;
ro(1)=0.01;

m=115;

Ix=6.1;
Iy=5.98;
Iz=5.517;
Ixy=-0.00016;
Ixz=-0.185;
Iyz=0.0006;

invMCD=0;
i=0;

for i = 1:20
    C11=zeros(3,3);
    C12=[0 m*wo(i) -m*vo(i); -m*wo(i) 0 m*uo(i); m*vo(i) -m*uo(i) 0];
    C21=-C12;
    C22=[0 -Iyz*qo(i)-Ixz*po(i)+Iz*ro(i)  Iyz*ro(i)+Ixy*po(i)-Iy*qo(i);...
        Iyz*qo(i)+Ixz*po(i)-Iz*ro(i) 0 -Ixz*ro(i)-Ixy*qo(i)+Ix*po(i); ...
        -Iyz*ro(i)-Ixy*po(i)+Iy*qo(i) Ixz*ro(i)+Ixy*qo(i)-Ix*po(i) 0];
   C=[C11 C12;C21 C22];

   invMCD(1:6,1:6,i)=-inv(M)*(C+DQL);
   diag_invMCD(1:6,1:6,i)=diag([invMCD(1,1) invMCD(2,2) invMCD(3,3) invMCD(4,4) invMCD(5,5) invMCD(6,6) ]);
   offdiag_invMCD(1:6,1:6,i)=invMCD(1:6,1:6,i)-diag_invMCD(1:6,1:6,i);
   sum_off_diag1=sum(abs(offdiag_invMCD(1,1:6,i)));
   sum_off_diag2=sum(abs(offdiag_invMCD(2,1:6,i)));
   sum_off_diag3=sum(abs(offdiag_invMCD(3,1:6,i)));
   sum_off_diag4=sum(abs(offdiag_invMCD(4,1:6,i)));
   sum_off_diag5=sum(abs(offdiag_invMCD(5,1:6,i)));
   sum_off_diag6=sum(abs(offdiag_invMCD(6,1:6,i)));
   i=i+1;
  uo(i)=uo(i-1)+0.01;
vo(i)=vo(i-1)+0.01;
wo(i)=wo(i-1)+0.01;
po(i)=po(i-1)+0.01;
qo(i)=qo(i-1)+0.01;
ro(i)=ro(i-1)+0.01;

end

diag_invMCD
sum_off_diag1
sum_off_diag2
sum_off_diag3
sum_off_diag4
sum_off_diag5
sum_off_diag6



% We try to make a vector of points for the circle:
N=256;
t=(0:N)*2*pi/N;

figure
plot( sum_off_diag1*cos(t)+1, sum_off_diag1*sin(t)+1 ,'-',diag_invMCD(1,1,1)*cos(t)+1,diag_invMCD(1,1,1)*sin(t)+1,'--');
hold on
plot( sum_off_diag2*cos(t)+2, sum_off_diag2*sin(t)+2 ,'-',diag_invMCD(2,2,1)*cos(t)+2,diag_invMCD(2,2,1)*sin(t)+2,'--');
hold on
plot( sum_off_diag3*cos(t)+3, sum_off_diag3*sin(t)+3 ,'-',diag_invMCD(3,3,1)*cos(t)+3,diag_invMCD(3,3,1)*sin(t)+3,'--');
hold on
plot( sum_off_diag4*cos(t)+4, sum_off_diag4*sin(t)+4 ,'-',diag_invMCD(4,4,1)*cos(t)+4,diag_invMCD(4,4,1)*sin(t)+4,'--');
hold on
plot( sum_off_diag5*cos(t)+5, sum_off_diag5*sin(t)+5 ,'-',diag_invMCD(5,5,1)*cos(t)+5,diag_invMCD(5,5,1)*sin(t)+5,'--');
hold on
plot( sum_off_diag6*cos(t)+6, sum_off_diag6*sin(t)+6 ,'-',diag_invMCD(6,6,1)*cos(t)+6,diag_invMCD(6,6,1)*sin(t)+6,'--');
title('Sum of Off-diagonal element vs diagonal element of M^{-1}(C(v)+D)for v \in [0, 0.1m/s)')
xlabel('row')
ylabel('columnn')
%xlabel('M^{-1}[C(v)+D] (in kg/s)')
%ylabel('M^{-1}[C(v)+D] (in kg/s)')
legend('radius = sum of off-diagonal element','radius = diagonal element')
axis([0 6 0 6])


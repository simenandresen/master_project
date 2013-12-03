

run Analy_ss;
disp('--------------------------------------');
disp('LQR Controller Design');
disp('--------------------------------------');

[len,width]=size(Bt);
[len2,width2]=size(Ct');

Q=Ct'*Ct*0.1;   
%Q(8:14,8:14)=diag([1,1,1,1 ,1,1,1]*0);
%Q=diag([10,2,0.3, 0.4, 0.5,     0.1 0.2 0.3 0.3 0.5,   0.1 0.2 0.1 0.3])*10;
r=diag([0.0002, 0.2,0.1,      0.3,0.1,0.8])*0.001;

f=lqr(At,Bt,Q,r)														% state feedback gain

[Alqr,Blqr,Clqr,Dlqr]=reg(At,Bt,Ct,Dt,f,zeros(len,width));  
sys_lqr=ss(Alqr,Blqr,Clqr,Dlqr);


disp('--------------------------------------');
disp('LQG Controller Design');
disp('--------------------------------------');

% LQR
Q=Ct'*Ct;
r=1*eye(width);														
f=lqr(At,Bt,Q,r)														% state feedback gain

% LQG
Qe=1*eye(len);
re=1*eye(width2);														   

fe=lqr(Aall',Call',Qe,re);
fe=fe'

[Alqg,Blqg,Clqg,Dlqg]=reg(At,Bt,Ct,Dt,f,fe);        % form KLQG
sys_lqg=ss(Alqg,Blqg,Clqg,Dlqg);


% LQG/LTR
disp('--------------------------------------');
disp('LQG/LTR Controller Design');
disp('--------------------------------------');



% f
Q=Ct'*Ct*0.1;   
%Q(8:14,8:14)=diag([1,1,1,1 ,1,1,1]*0);
%Q=diag([10,2,0.3, 0.4, 0.5,     0.1 0.2 0.3 0.3 0.5,   0.1 0.2 0.1 0.3])*10;
r=diag([0.0002, 0.2,0.1,      0.3,0.1,0.8])*0.001;

% width=6, len=14

f=lqr(At,Bt,Q,r)														% state feedback gain

Qe1=diag([0.1,0.2,0.3, 0.4, 0.5,     0.1 0.2 0.3 0.3 0.5,   0.1 0.2 0.1 0.3])*10;
Qe= (0.01*Bt*Bt');                     										   
re=diag([0.002, 0.2,0.1,      0.3,0.1,0.8])*0.001;


fe=lqr(At',Ct',Qe,re);
fe=fe'

[Alqgltr,Blqgltr,Clqgltr,Dlqgltr]=reg(At,Bt,Ct,Dt,f,fe);         % form KLQG/LTR
sys_lqgltr=ss(Alqgltr,Blqgltr,Clqgltr,Dlqgltr);
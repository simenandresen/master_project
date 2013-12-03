clc;
disp('--------Error Eclidean Norm------------');
n_x1p=sqrt(sum(x1p.^2))
n_x2p=sqrt(sum(x2p.^2))
n_x1e=sqrt(sum(x1e.^2))

disp('--------Thrust Eclidean Norm------------');
n_thrust=sqrt(sum(thrust.^2))

disp('--------Stability- V1_dot-----------');
d1=1*eye(6);% P controller
d2=-0.01*eye(6); %K02*Jinv(pi/2)
d3=inv(M-Ma)*n_thrust';
V1dot=DQL*(n_x2p'.^2)+d2*Ma*n_x1p'.^2-d1*Ma*n_x2p'-Ma*d3

disp('--------Stability- V2_dot-----------');
d5=eye(6);%P controller
K01=0.1;
V2dot=-eye(6)*0.1*Ma*(n_x1p.*n_x1e)'-K01*Ma*(n_x1p.*n_x1e)'

disp('--------Settling time- velocity @ 0 m/s(proposed)-----------');
[u,ui]=min(abs(boutd(2:length(boutd),1)));
[v,vi]=min(abs(boutd(2:length(boutd),2)));
[w,wi]=min(abs(boutd(2:length(boutd),3)));
[p,pi]=min(abs(boutd(2:length(boutd),4)));
[q,qi]=min(abs(boutd(2:length(boutd),5)));
[r,ri]=min(abs(boutd(2:length(boutd),6)));
time_u=t(ui);
time_v=t(vi);
time_w=t(wi);
time_p=t(pi);
time_q=t(qi);
time_r=t(ri);

disp(['Settling time for u=',num2str(time_u),' sec'] );
disp(['Settling time for v=',num2str(time_v),' sec'] );
disp(['Settling time for w=',num2str(time_w),' sec'] );
disp(['Settling time for p=',num2str(time_p),' sec'] );
disp(['Settling time for q=',num2str(time_q),' sec'] );
disp(['Settling time for r=',num2str(time_r),' sec'] );

disp('--------Settling time- velocity @ 0 m/s(backstepping)-----------');
[u,ui]=min(abs(boutd(2:length(boutbk),1)));
[v,vi]=min(abs(boutd(2:length(boutbk),2)));
[w,wi]=min(abs(boutd(2:length(boutbk),3)));
[p,pi]=min(abs(boutd(2:length(boutbk),4)));
[q,qi]=min(abs(boutd(2:length(boutbk),5)));
[r,ri]=min(abs(boutd(2:length(boutbk),6)));
time_u=t(ui);
time_v=t(vi);
time_w=t(wi);
time_p=t(pi);
time_q=t(qi);
time_r=t(ri);

disp(['Settling time for u=',num2str(time_u),' sec'] );
disp(['Settling time for v=',num2str(time_v),' sec'] );
disp(['Settling time for w=',num2str(time_w),' sec'] );
disp(['Settling time for p=',num2str(time_p),' sec'] );
disp(['Settling time for q=',num2str(time_q),' sec'] );
disp(['Settling time for r=',num2str(time_r),' sec'] );

disp('--------Settling time- velocity @ 0 m/s(P-P)-----------');
[u,ui]=min(abs(boutd(2:length(boutpp),1)));
[v,vi]=min(abs(boutd(2:length(boutpp),2)));
[w,wi]=min(abs(boutd(2:length(boutpp),3)));
[p,pi]=min(abs(boutd(2:length(boutpp),4)));
[q,qi]=min(abs(boutd(2:length(boutpp),5)));
[r,ri]=min(abs(boutd(2:length(boutpp),6)));
time_u=t(ui);
time_v=t(vi);
time_w=t(wi);
time_p=t(pi);
time_q=t(qi);
time_r=t(ri);

disp(['Settling time for u=',num2str(time_u),' sec'] );
disp(['Settling time for v=',num2str(time_v),' sec'] );
disp(['Settling time for w=',num2str(time_w),' sec'] );
disp(['Settling time for p=',num2str(time_p),' sec'] );
disp(['Settling time for q=',num2str(time_q),' sec'] );
disp(['Settling time for r=',num2str(time_r),' sec'] );



i=0; 
disp('--------Settling time- pos (proposed)-----------');
for i=1:length(eoutd)
if eoutd(i,1)>=1*0.8 & eoutd(i,1)<=1.2
    ts_d=t(i);
     disp(['Settling time for x = ',num2str(ts_d),' sec'] );
     break
end
    i=i+1;
end

for i=1:length(eoutd)
if eoutd(i,3)>=1*0.8 & eoutd(i,3)<=1.2
    ts_d=t(i);
     disp(['Settling time for z = ',num2str(ts_d),' sec'] );
     break
end
    i=i+1;
end

for i=1:length(eoutd)
if eoutd(i,6)>=1*0.8 & eoutd(i,6)<=1.2
    ts_d=t(i);
     disp(['Settling time for yaw angle = ',num2str(ts_d),' sec'] );
     break
end
    i=i+1;
end


disp('--------Settling time- pos (backstepping)-----------');
for i=1:length(eoutbk)
if eoutd(i,1)>=1*0.8 & eoutd(i,1)<=1.2
    ts_bk=t(i);
     disp(['Settling time for x = ',num2str(ts_bk),' sec'] );
     break
end
    i=i+1;
end

for i=1:length(eoutbk)
if eoutd(i,3)>=1*0.8 & eoutd(i,3)<=1.2
    ts_bk=t(i);
     disp(['Settling time for z = ',num2str(ts_bk),' sec'] );
     break
end
    i=i+1;
end

for i=1:length(eoutbk)
if eoutd(i,6)>=1*0.8 & eoutd(i,6)<=1.2
    ts_bk=t(i);
     disp(['Settling time for yaw angle = ',num2str(ts_bk),' sec'] );
     break
end
    i=i+1;
end
    
 disp('--------Settling time- pos P-P)-----------');
for i=1:length(eoutpp)
if eoutd(i,1)>=1*0.8 & eoutd(i,1)<=1.2
    ts_pp=t(i);
     disp(['Settling time for x = ',num2str(ts_pp),' sec'] );
     break
end
    i=i+1;
end

for i=1:length(eoutpp)
if eoutd(i,3)>=1*0.8 & eoutd(i,3)<=1.2
    ts_pp=t(i);
     disp(['Settling time for z = ',num2str(ts_pp),' sec'] );
     break
end
    i=i+1;
end

for i=1:length(eoutpp)
if eoutd(i,6)>=1*0.8 & eoutd(i,6)<=1.2
    ts_pp=t(i);
     disp(['Settling time for yaw angle = ',num2str(ts_pp),' sec'] );
     break
end
    i=i+1;
end
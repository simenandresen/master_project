%Model-Order Reduction for RRC ROV & Nonlinear ROV Analysis

%Initialize
clc;
cd c:\Matlab7\toolbox\rov_design_analysis\mat_file;
load RRCRRC22.mat
i=0;
t=100; % (sec)
n=t+1;
K=eye(6);
N=5; %no of loops

cd c:\Matlab7\toolbox\rov_design_analysis\hybrid ;
sim('MOR')

%Initial Error norm 
error=bout(:,1:6)-bout1(:,1:6);
errorsum=sqrt(sum(error.^2))

%Loops
for j=1:N 
KK=bout(n,:)./bout1(n,:);
K=diag(KK)+ 0.1* diag([1 1 1.5 0 0 1]); 
sim('MOR');
error=bout(:,1:6)-bout1(:,1:6);
errorsum(j,1:6)=sqrt(sum(error.^2))

%model reduction- check whether it preserve the gramians
velbar=mean(bout(:,1:6));
errb4=bout(:,1:6)- ones(length(bout),1)*velbar;

velbar=mean(bout1(:,1:6));
erra=bout(:,1:6)- ones(length(bout1),1)*velbar;

posbar=mean(eout(:,1:6));
perrb4=eout(:,1:6)- ones(length(eout),1)*posbar;

posbar=mean(eout1(:,1:6));
perra=eout(:,1:6)- ones(length(eout1),1)*posbar;

% Y
Yb=errb4'*errb4; 
eigRb=eig(Yb);
Ya=erra(:,:)'*erra(:,:); 
eigRa=eig(Ya);detYa(j)=det(Ya)

%X
Xb=perrb4'*perrb4; 
eigRb=eig(Xb);
Xa=perra(:,:)'*perra(:,:);
eigRa=eig(Xa);detXa(j)=det(Xa)

%check the L2 gain
ssum_u1=sqrt(sum(u1.^2));
ssum_bout1=sqrt(sum(bout1.^2));
gamma_a(j)=ssum_bout1/ssum_u1

ssum_u=sqrt(sum(u.^2));
ssum_bout=sqrt(sum(bout.^2));
gamma_b(j)=ssum_bout/ssum_u
j=j+1
end

K    
disp('Completed')


%Plot convergence graphs
figure(3)
subplot(3,1,1)
plot(1:N, errorsum(:,1))
subplot(3,1,2)
plot(1:N, errorsum(:,2))
subplot(3,1,3)
plot(1:N, errorsum(:,3))

figure(4)
subplot(3,1,1)
plot(1:N, errorsum(:,4))
subplot(3,1,2)
plot(1:N, errorsum(:,5))
subplot(3,1,3)
plot(1:N, errorsum(:,6))


%Compare full and reduced order model
sim('MOR');
nonlin_resp6_comp




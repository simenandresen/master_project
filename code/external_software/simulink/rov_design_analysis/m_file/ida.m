for i=1:length(t)
    z1(i)=z(1,1,i);
    z2(i)=z(2,1,i);
    x1(i)=x(1,1,i);
    x2(i)=x(2,1,i);
end

clc
figure
plot(t,z1,'--',t,z2,'-')
xlabel('Time(in sec)')
ylabel('z')
legend('row1','row 2')
figure
plot(t,x1,'--',t,x2,'-')
xlabel('Time(in sec)')
ylabel('x')
legend('row1','row 2')

disp('closed loop with R=[5 0;0 0], u=k*x=B^T*P*x ')
J=[1 -2;2 3];
R=[5 0;0 1]
H=[3 0; 0 4];
B=[0 1]';
alpha=2;
eig_JRH=eig((J-R)*H-alpha*B*B'*H)
normZW=sqrt(sum(z1.^2+z2.^2))/sqrt(sum(w.^2))


disp('closed loop with R=[5 0;0 2], u=k*x=alpha*B^T*Hx ')
Rtilde=[5 0;0 5] 
eig_JHR_tilde= eig((J-Rtilde)*H-alpha*B*B'*H)
normZW=sqrt(sum(z1.^2+z2.^2))/sqrt(sum(w.^2))

disp('open loop ')
eig_JRH=eig((J-R)*H)

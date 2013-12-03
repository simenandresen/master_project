
clc 
close all
clear

cd c:\Matlab7\toolbox\rov_design_analysis\\mat_file 
load rrcrovf      
cd c:\Matlab7\toolbox\rov_design_analysis\hybrid 

uc1=5;
uc2=-5;
uc3=0;
uc4=0;

for i=1:14
      sim('controllable_property')
      
      uc1=uc1+5
      uc2=uc2-5
      uc3=0
      uc4=0
           
      
      unact_vel(:,:,i)=bout(:,4:5);
      unact_pos(:,:,i)=eout(:,4:5);
      max_unact_vel1(i)=max(bout(:,4));   % roll
      max_unact_vel2(i)=max(bout(:,5));   % pitch
      final_unact_pos1(i)=eout(length(t),4);
      final_unact_pos2(i)=eout(length(t),5);
      
      
      act_vel(:,:,i)=bout(:,[1:3,6]);
      act_pos(:,:,i)=eout(:,[1:3,6]);
      max_act_vel1(i)=max(bout(:,1)); % u
      max_act_vel2(i)=max(bout(:,2)); % v
      max_act_vel3(i)=max(bout(:,3)); % w
      max_act_vel4(i)=max(bout(:,6)); % r

      final_act_pos1(i)=eout(length(t),[1]);
      final_act_pos2(i)=eout(length(t),[2]);
      final_act_pos3(i)=eout(length(t),[3]);
      final_act_pos4(i)=eout(length(t),[6]);
end

figure
plot(t,unact_vel(:,1,1),'y:')
hold on
plot(t,unact_vel(:,1,2),'-')
plot(t,unact_vel(:,1,3),':')
plot(t,unact_vel(:,1,4),'-.')
plot(t,unact_vel(:,1,5),'--')
plot(t,unact_vel(:,1,6),'b')
plot(t,unact_vel(:,1,7),'g')
plot(t,unact_vel(:,1,8),'r')
plot(t,unact_vel(:,1,9),'c')
plot(t,unact_vel(:,1,10),'m')
plot(t,unact_vel(:,1,11),'y')
plot(t,unact_vel(:,1,12),'k')
plot(t,unact_vel(:,1,13),'b:')
plot(t,unact_vel(:,1,14),'r:')
title('Roll rate at different u(t)')
xlabel('Time (in sec)')
ylabel('Roll rate (in rad/s)')
legend('u=5','u=10','u=15','u=20','u=25','u=30','u=35','u=40','u=45','u=50'...
    ,'u=55','u=60','u=65','u=70')

figure
plot(t,unact_vel(:,2,1),'y:')
hold on
plot(t,unact_vel(:,2,2),'-')
plot(t,unact_vel(:,2,3),':')
plot(t,unact_vel(:,2,4),'-.')
plot(t,unact_vel(:,2,5),'--')
plot(t,unact_vel(:,2,6),'b')
plot(t,unact_vel(:,2,7),'g')
plot(t,unact_vel(:,2,8),'r')
plot(t,unact_vel(:,2,9),'c')
plot(t,unact_vel(:,2,10),'m')
plot(t,unact_vel(:,2,11),'y')
plot(t,unact_vel(:,2,12),'k')
plot(t,unact_vel(:,2,13),'b:')
plot(t,unact_vel(:,2,14),'r:')
title('Pitch rate at different u(t)')
xlabel('Time (in sec)')
ylabel('Pitch rate (in rad/s)')
legend('u=5','u=10','u=15','u=20','u=25','u=30','u=35','u=40','u=45','u=50'...
    ,'u=55','u=60','u=65','u=70')


figure
plot(t,unact_pos(:,1,1),'y:')
hold on
plot(t,unact_pos(:,1,2),'-')
plot(t,unact_pos(:,1,3),':')
plot(t,unact_pos(:,1,4),'-.')
plot(t,unact_pos(:,1,5),'--')
plot(t,unact_pos(:,1,6),'b')
plot(t,unact_pos(:,1,7),'g')
plot(t,unact_pos(:,1,8),'r')
plot(t,unact_pos(:,1,9),'c')
plot(t,unact_pos(:,1,10),'m')
plot(t,unact_pos(:,1,11),'y')
plot(t,unact_pos(:,1,12),'k')
plot(t,unact_pos(:,1,13),'b:')
plot(t,unact_pos(:,1,14),'r:')
title('Roll angle at different u(t)')
xlabel('Time (in sec)')
ylabel('Roll angle (in rad)')
legend('u=5','u=10','u=15','u=20','u=25','u=30','u=35','u=40','u=45','u=50'...
    ,'u=55','u=60','u=65','u=70')


figure
plot(t,unact_pos(:,2,1),'y:')
hold on
plot(t,unact_pos(:,2,2),'-')
plot(t,unact_pos(:,2,3),':')
plot(t,unact_pos(:,2,4),'-.')
plot(t,unact_pos(:,2,5),'--')
plot(t,unact_pos(:,2,6),'b')
plot(t,unact_pos(:,2,7),'g')
plot(t,unact_pos(:,2,8),'r')
plot(t,unact_pos(:,2,9),'c')
plot(t,unact_pos(:,2,10),'m')
plot(t,unact_pos(:,2,11),'y')
plot(t,unact_pos(:,2,12),'k')
plot(t,unact_pos(:,2,13),'b:')
plot(t,unact_pos(:,2,14),'r:')
title('Pitch angle at different u(t)')
xlabel('Time (in sec)')
ylabel('Pitch angle (in rad)')
legend('u=5','u=10','u=15','u=20','u=25','u=30','u=35','u=40','u=45','u=50'...
    ,'u=55','u=60','u=65','u=70')


clc;
velf=sum(bout(:,1:6)'*bout(:,1:6));
posf=sum(eout(:,1:6)'*eout(:,1:6));
format long e
disp('---Controllability --vel---')
velf
disp('---Controllability --pos---')
posf
figure
subplot(2,1,1)
bar(1:1:6,velf')
title('2-norm at each velocity')
xlabel('Velocity')
ylabel('2-norm')
subplot(2,1,2)
bar(1:1:6,posf')
title('2-norm at each Position')
xlabel('Position')
ylabel('2-norm')
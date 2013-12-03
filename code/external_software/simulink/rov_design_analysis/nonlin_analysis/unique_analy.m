% Nonlinear analysis- uniqueness/existence
clc
fx_fy=[fx_fy1 fx_fy2];
norm_fx_fy=sqrt(sum(fx_fy.^2))
norm_xy=sqrt(sum(xy.^2))
L=norm_fx_fy./norm_xy
disp('Is ||f(x_s_1)-f(x_s_2)|| < L||x_s_1-x_s_2||, where Lipschitz constant > 0')
L>0


figure(5)
plot(1:1:8,L(:,[1:4,7:10]))
xlabel('states')
ylabel('L')
title('Lipschitz constant')


% Nonlinear analysis- continuous parameters dependence
fx_fy_uc=[fx_fy_uc1 fx_fy_uc2];
norm_fx_fy_uc=sqrt(sum(fx_fy_uc.^2))
norm_xy_uc=sqrt(sum(xy_uc.^2))

disp('-----For constant uc-----')
disp('Is ||xa1-xa2|| < delta, where delta > 0')
norm_xy>0
disp('Is ||f(x_s_1)-f(x_s_2)|| < eilpson, where eilpson > 0')
norm_fx_fy>0

disp('-----For constant x-----')
disp('Is ||x_s_1-x_s_2|| < delta, where delta > 0')
norm_xy_uc>0
disp('Is ||f(x_s_1)-f(x_s_2)|| < eilpson, where eilpson > 0')
norm_fx_fy_uc>0

figure(6)
subplot(2,1,1)
plot(1:1:8,norm_xy(:,[1:4,7:10]),'--',1:1:8,norm_fx_fy(:,[1:4,7:10]),'-')
xlabel('states(x_s)')
ylabel('2-norm')
title('norm of states x_s and system f(x_s)- constant u_bar')
legend('||x_s_1-x_s_2||','||f(x_s_1)-f(x_s_2)||')

subplot(2,1,2)
plot(1:1:8,norm_xy_uc(:,[1:4,7:10]),'--',1:1:8,norm_fx_fy_uc(:,[1:4,7:10]),'-')
xlabel('states(x_s)')
ylabel('2-norm')
title('norm of states x_s and system f(x_s)- constant x_s')
legend('||x_s_1-x_s_2||','||f(x_s_1)-f(x_s_2)||')


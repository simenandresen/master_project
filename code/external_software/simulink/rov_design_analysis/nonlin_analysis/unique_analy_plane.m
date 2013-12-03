% Nonlinear analysis- uniqueness/existence
clc
fx_fy=[fx_fy1 fx_fy2];
norm_fx_fy=sqrt(sum(fx_fy.^2))
norm_xy=sqrt(sum(xy.^2))
L=norm_fx_fy./norm_xy
disp('Is ||f(x_s_1)-f(x_s_2)|| < L||x_s_1-x_s_2||, where Lipschitz constant > 0')
L>0


figure
plot(1:1:6,L(:,[1:2,6,7:8,12]))
xlabel('states')
ylabel('L')
title('Lipschitz constant')


% Nonlinear analysis- continuous parameters dependence
fx_fy_uc=[fx_fy_uc1 fx_fy_uc2];
norm_fx_fy_uc=sqrt(sum(fx_fy_uc.^2))
norm_xy_uc=sqrt(sum(xy_uc.^2))

disp('-----For constant uc-----')
disp('Is ||x_h_1-x_h_2|| < delta, where delta > 0')
norm_xy>0
disp('Is ||f(x_h_1)-f(x_h_2)|| < eilpson, where eilpson > 0')
norm_fx_fy>0

disp('-----For constant x-----')
disp('Is ||x_h_1-x_h_2|| < delta, where delta > 0')
norm_xy_uc>0
disp('Is ||f(x_h_1)-f(x_h_2)|| < eilpson, where eilpson > 0')
norm_fx_fy_uc>0

figure 
subplot(2,1,1)
plot(1:1:6,norm_xy(:,[1:2,6,7:8,12]),'--',1:1:6,norm_fx_fy(:,[1:2,6,7:8,12]),'-')
xlabel('states(x_h)')
ylabel('2-norm')
title('norm of states x and system f(x)- constant u_bar')
legend('||x_h_1-x_h_2||','||f(x_h_1)-f(x_h_2)||')

subplot(2,1,2)
plot(1:1:6,norm_xy_uc(:,[1:2,6,7:8,12]),'--',1:1:6,norm_fx_fy_uc(:,[1:2,6,7:8,12]),'-')
xlabel('states(x_h)')
ylabel('2-norm')
title('norm of states x_h and system f(x_h)- constant x_h')
legend('||x_h_1-x_h_2||','||f(x_h_1)-f(x_h_2)||')



disp('-----For constant uc-----')
disp('Is ||x_v_1-x_v_2|| < delta, where delta > 0')
norm_xy>0
disp('Is ||f(x_v_1)-f(x_v_2)|| < eilpson, where eilpson > 0')
norm_fx_fy>0

disp('-----For constant x-----')
disp('Is ||x_v_1-x_v_2|| < delta, where delta > 0')
norm_xy_uc>0
disp('Is ||f(x_v_1)-f(x_v_2)|| < eilpson, where eilpson > 0')
norm_fx_fy_uc>0


figure 
subplot(2,1,1)
plot(1:1:6,norm_xy(:,[1:2,6,7:8,12]),'--',1:1:6,norm_fx_fy(:,[1:2,6,7:8,12]),'-')
xlabel('states(x_v)')
ylabel('2-norm')
title('norm of states x_v and system f(x_v)- constant u_bar')
legend('||x_v_1-x_v_2||','||f(x_v_1)-f(x_v_2)||')

subplot(2,1,2)
plot(1:1:6,norm_xy_uc(:,[1:2,6,7:8,12]),'--',1:1:6,norm_fx_fy_uc(:,[1:2,6,7:8,12]),'-')
xlabel('states(x_v)')
ylabel('2-norm')
title('norm of states x_v and system f(x_v)- constant x_v')
legend('||x_v_1-x_v_2||','||f(x_v_1)-f(x_v_2)||')


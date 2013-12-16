%-----------------------------------
%
%  main file
%  all code are called from here
%
%-----------------------------------

%initialization
clc;clear all;close all;
main_dir = '/home/simena/Dropbox/master_project/code/matlab/';
addpath(strcat(main_dir , 'gnc_mfiles'));
kinematic_parameters;


%% simulation
sim('kinematics.mdl',7);



%% save log
time = commanded_states.zeta_c.Time;

%% commanded trajectory
zeta_c = commanded_states.zeta_c.Data;
eta1_c = measured_states.xi.eta1.Data;
eta2_c = measured_states.xi.eta2.Data;
q_c = measured_states.xi.q.Data;
ee_vel_c = path_c.v_ee_c.Data;
ee_eta1_c = path_c.ee_pose.ee_pos.Data;
ee_eta2_c= path_c.ee_pose.ee_angle.Data;


eta1=eta1_c;
eta2=eta2_c;
q=q_c;
ee_eta1=ee_eta1_c;
ee_eta2=ee_eta2_c;

% eta1 = measured_states.xi.eta1.Data;
% eta2 = measured_states.xi.eta2.Data;
% q = measured_states.xi.q.Data;
% zeta = measured_states.zeta.Data;
% 
% ee_eta1 = measured_states.ee.ee_eta1.Data;
% ee_eta2 = measured_states.ee.ee_eta2.Data;
% ee_vel = measured_states.ee.ee_vel.Data;

x_ee_in_b=x_ee_in_body.Data;
H_trace = H_matrix.Data;


myanimation;
myplotting;
















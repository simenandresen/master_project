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
sim('dynamics_kinematics',6);



%% save log
time = commanded_states.zeta_c.Time;

%% commanded trajectory
zeta_c = commanded_states.zeta_c.Data;
eta1_c = commanded_states.xi_c.eta1.Data;
eta2_c = commanded_states.xi_c.eta2.Data;
q_c = commanded_states.xi_c.q.Data;
ee_vel_c = path_c.v_ee_c.Data;
ee_eta1_c = path_c.ee_pose.ee_pos.Data;
ee_eta2_c= path_c.ee_pose.ee_angle.Data;


% zeta_c = zeta_c_s.signals.values;
% eta1_c = eta1_c_s.signals.values;
% eta2_c = eta2_c_s.signals.values;
% q_c = q_c_s.signals.values;
% ee_vel_c = ee_vel_c_s.signals.values;
% ee_eta1_c = ee_eta1_c_s.signals.values;
% ee_eta2_c= ee_eta2_c_s.signals.values;


%% measured trajectories
% eta1 = xi_s.signals.values(:,1:3);
% eta2 = xi_s.signals.values(:,4:6);
% q = xi_s.signals.values(:,7:12);
% zeta = zeta_s.signals.values;
% 
% ee_eta1 = ee_eta1_s.signals.values;
% ee_eta2 = ee_eta2_s.signals.values;
% ee_vel = ee_vel_s.signals.values;

eta1 = measured_states.xi.eta1.Data;
eta2 = measured_states.xi.eta2.Data;
q = measured_states.xi.q.Data;
zeta = measured_states.zeta.Data;

ee_eta1 = measured_states.ee.ee_eta1.Data;
ee_eta2 = measured_states.ee.ee_eta2.Data;
ee_vel = measured_states.ee.ee_vel.Data;

%% force end effector

f_config = force_ee.f_config.Data;
f_ee     = force_ee.f_ee.Data;
tau_c = measured_states.tau_c.Data;

alpha=ee_pos_corrector.alpha.Data;
pos_corr=ee_pos_corrector.pos_corr.Data;
force_norm=ee_pos_corrector.force_norm.Data;



















%Model-Order Reduction for RRC ROV & Nonlinear ROV Analysis

%Initialize
clc;
cd c:\Matlab7\toolbox\rov_design_analysis\mat_file;
load rrcrovf.mat

Kc=[eye(3) zeros(3,3);zeros(2,6); [0 0 0 0 0 1]]

cd c:\Matlab7\toolbox\rov_design_analysis\hybrid ;
sim('rov_property')

cd c:\Matlab7\toolbox\rov_design_analysis\m_file
run nonlin_resp6_comp

disp('----------------------invM * T-------------------')
invMT=inv(M)*T

disp('----------------------invM * T * Kc  -------------------')
invMTKc=inv(M)*Kc*T

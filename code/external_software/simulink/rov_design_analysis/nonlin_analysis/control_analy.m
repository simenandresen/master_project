% Nonlinear analysis- controllability

%Full-order
clc;
velf=sum(bout1(:,1:6)'*bout1(:,1:6));
posf=sum(eout1(:,1:6)'*eout1(:,1:6));

%Station-keeping
vels=sum(bout1(:,1:4)'*bout1(:,1:4));
poss=sum(eout1(:,1:4)'*eout1(:,1:4));

%horizontal
velh=sum(bout1(:,[1:2,6])'*bout1(:,[1:2,6]));
posh=sum(eout1(:,[1:2,6])'*eout1(:,[1:2,6]));

%vertical
velv=sum(bout1(:,[1,3,5])'*bout1(:,[1,3,5]));
posv=sum(eout1(:,[1,3,5])'*eout1(:,[1,3,5]));

disp(' Y & X -full order')
format long e
disp('---Controllability Gramians--vel---')
velf
disp('---Controllability Gramians--pos---')
posf


disp(' Y & X -station keeping')
disp('---Controllability Gramians--vel---')
vels
disp('---Controllability Gramians--pos---')
poss


disp(' Y & X -horizontal')
disp('---Controllability Gramians--vel---')
velh
disp('---Controllability Gramians--pos---')
posh

disp(' Y & X -vertical')
disp('---Controllability Gramians--vel---')
velv
disp('---Controllability Gramians--pos---')
posv

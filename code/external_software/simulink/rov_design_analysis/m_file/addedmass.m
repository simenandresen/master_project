function [txudot,tyvdot,tzwdot,tkpdot,tmqdot,tnrdot]=addedmass(Dr,Lr,ps)
%%Determine the added mass of RRC ROV2 in 6 directions using Strip theory
%%
%%[txudot,tyvdot,tzwdot,tkpdot,tmqdot,tnrdot]=addedmass(Dr,Lr,ps)
%%where 
%%Dr= [0.09 0.22 0.22 0.12 0.12];
%%Lr=[0.2 0.9 0.9 0.92 0.92];
%%ps=1024;
%%-----------------------------------------------------------
%%Description:
%%Dr- dia (in m) of thruster, left & right barrel, left and right float (1 x 5).
%%Lr- length (in m) of thruster, left & right barrel, left and right float (1 x 5).
%%ps- density (=1024 kg/m^3) of the seawater (3.5% salinity) at 20 deg cel.

for i=1:5
yvdot(1,i)=-pi*ps*(0.5*Dr(1,i))^2;
zwdot(1,i)=-pi*ps*(0.5*Dr(1,i))^2;
kpdot(1,i)=-1/48*pi*ps*(0.5*Dr(1,i))^5;
mqdot(1,i)=-1/12*((pi*ps*(0.5*Dr(1,i))^2*Lr(1,i)^3)+0.1*Dr(1,i)^2);
nrdot(1,i)=-1/12*((pi*ps*(0.5*Dr(1,i))^2*Lr(1,i)^3)+0.1*Dr(1,i)^2);
i-i+1;
end

txudot=-0.1*6;
tyvdot=sum(yvdot);
tzwdot=sum(zwdot);
tkpdot=sum(kpdot);
tmqdot=sum(mqdot);
tnrdot=sum(nrdot);




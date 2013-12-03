function sol=cable3dbvp(EndP,L)
W=[0 0 -1]'; %Constant distributed force
Fguess=[4 5 180]'; %Guess for end force EndP (We know this guess is wrong)
E=200e9; %Modulus of elasticity
D=0.014; %Cable diameter
A=pi*(D/2)^2; %Cable cross-sectional area
L=300;

%EndP=Boundary point for second end.
s=0:.1:L; %Initial mesh
%Calculates an initial guess for the cable's geometry (a straight line)
n=length(s);
Xguess=linspace(0,EndP(1),n);
Yguess=linspace(0,EndP(2),n);
Zguess=linspace(0,EndP(3),n);
sol.x=s;
sol.y=[Xguess; Yguess; Zguess];
sol.parameters=Fguess;
%Plots the initial guess
figure(1)
clf
plot(sol.y(1,:),sol.y(3,:),'k--')
xlabel('x')
ylabel('z')
title('Cable bvp')
sol = bvp4c(@cableODE,@cableBC,sol,[],W,E,A,L,EndP); %Solves the BVP
%Plots the result
hold on
plot(sol.y(1,:),sol.y(3,:),'k-')
%new insertion
hold off
disp(sprintf('Calculated end force is [%1.2f %1.2f %1.2f] N \n',sol.parameters(1),sol.parameters(2),sol.parameters(3)))
%System's ODEs

function drds = cableODE(s,r,F,W,E,A,L,EndP)
Fk=F+W*L;
drds = (Fk-W*s)*(1/sqrt((Fk-W*s)'*(Fk-W*s))+1/(E*A));
%Systems boundary values
 
function res = cableBC(ya,yb,F,W,E,A,L,EndP)
res = [ ya(1)
ya(2)
ya(3)
yb(1)-EndP(1)
yb(2)-EndP(2)
yb(3)-EndP(3)];
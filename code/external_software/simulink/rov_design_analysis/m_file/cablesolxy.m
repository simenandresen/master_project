
function [Fx,Fy]=cablesolxy
clear all
z=0; %height of the finite segment wrt start point
L=300; % finite segment of the cable
x=0;
y=0;

n=10;
m=10;

for j=1:m
    j
    EndP=[x 0 300]'; %assume x and y position of the start point remains unchanged
    [sol]=cable3dbvp(EndP,L);
    Fguess=[sol.parameters(1) sol.parameters(2) sol.parameters(3)]
    Fx(j,1:length(sol.parameters))=Fguess;
    x=x+1;
end

for i=1:n
    i
    EndP=[0 y 300]'; %assume x and y position of the start point remains unchanged
    [sol]=cable3dbvp(EndP,L);
    Fguess=[sol.parameters(1) sol.parameters(2) sol.parameters(3)]
    Fy(i,1:length(sol.parameters))=Fguess;
    y=y+1;
end

figure(3)
subplot(3,1,1)
plot(1:length(Fx),Fx(:,1))
ylabel('Cable Force, Fx(N)')
xlabel('x(m)')

subplot(3,1,2)
plot(1:length(Fx),Fx(:,2))
ylabel('Cable Force,Fy(N)')
xlabel('x(m)')

subplot(3,1,3)
plot(1:length(Fx),Fx(:,3))
ylabel('Cable Force,Fz(N)')
xlabel('x(m)')

figure(4)
subplot(3,1,1)
plot(1:length(Fy),Fy(:,1))
ylabel('Cable Force, Fx(N)')
xlabel('y(m)')

subplot(3,1,2)
plot(1:length(Fy),Fy(:,2))
ylabel('Cable Force,Fy(N)')
xlabel('y(m)')

subplot(3,1,3)
plot(1:length(Fy),Fy(:,3))
ylabel('Cable Force,Fz(N)')
xlabel('y(m)')

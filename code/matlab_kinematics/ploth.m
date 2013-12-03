clc;
clear all;
kinematic_parameters;
i=1;

q=qmin(i):0.01:qmax(i);

for j=1:length(q)
    a(j)= (1/4)*(-(qmax(i) - qmin(i)).^2)/((-q(j) + qmax(i))*(q(j) - qmin(i)).^2) + (qmax(i) - qmin(i)).^2/((-q(j) + qmax(i)).^2*(q(j) - qmin(i)));
end

plot(q)
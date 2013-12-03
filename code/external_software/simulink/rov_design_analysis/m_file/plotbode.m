function [mag, phase,wm]=plotbode
%Bode frequency response of LTI models on mag and phase data file from DSA.
%[mag,phase,wm]=plotbode
%   c. S. Chin 18-04-2006
%   Copyright 2006  

[filename,filepath]=uigetfile('*.*','Load Mag data file *.mat');
file=sprintf('%s\%s',filepath,filename);
fid=fopen(file);
fprintf('\n %s\t',filename)
load(filename)

for i=1:length(y)
    wm(i)=x(i);
    mag(i)=20*log10(abs(y(i)));
end

[filename,filepath]=uigetfile('*.*','Load phase data file *.mat');
file=sprintf('%s\%s',filepath,filename);
fid=fopen(file);
fprintf('\n %s\t',filename)
load(filename)

for j=1:length(y)
    wp(j)=x(j);
    phase(j)=atan((real(y(j))/imag(y(j)))).*180/pi;
end

figure(1)
subplot(2,1,1)
semilogx(wm,mag)
title('Bode Plot')
ylabel ('dB')
xlabel('Frequency (rad/s)')
subplot(2,1,2)
semilogx(wp,phase)
ylabel ('Phase(deg)')
xlabel('Frequency (rad/s)')

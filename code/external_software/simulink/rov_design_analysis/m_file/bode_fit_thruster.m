
w=logspace(-2, 4, 60);

for i = 1:length(w)
   frtest=w(i);
   sim('freq_thruster');
   ma(i)=mag1(length(mag1),1);
   ph(i)=phase1(length(phase1),1);
end



% nonlinear ROV Bode
figure(12); 
subplot(2,1,1);
semilogx(w,(ma));
title('Bode plot')
ylabel('Magnitude(dB)')
subplot(2,1,2);
semilogx(w,ph);
ylabel('Phase(deg)');
xlabel('Frequency(rad/s)');

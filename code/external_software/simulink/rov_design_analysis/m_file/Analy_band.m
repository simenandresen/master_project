% bode plot- check bandwidth
ww=logspace(-2,3,100);

for i= 1: length(ww)
Giw=evalfr(sys_tot,ww(i));									% GK(iw) where K=1
sv_Giw(1:6,i)=20*log10(svd(Giw*Giw'));                        
G1(i)= sv_Giw(1,i);G2(i)= sv_Giw(2,i);G3(i)= sv_Giw(3,i); % prepare sv data
G4(i)= sv_Giw(4,i);G5(i)= sv_Giw(5,i);G6(i)= sv_Giw(6,i);
end

figure(3);clf
loglog(ww,G1,ww,G2,ww,G3,ww,G4,ww,G5,ww,G6 );
hold on
loglog(ww,1);
axis([min(ww) max(ww) min(min(sv_Giw)) max(max(sv_Giw))]);
title('svd(G) ')
ylabel('Singular Values(dB)')
xlabel('Frequency (rad/s)')
legend('G1','G2','G3','G4','G5','G6');
hold off











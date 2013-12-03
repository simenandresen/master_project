function Cr=drag(mr,mo,lo,lr)
%%Determine the quadratic drag forces of RRC ROV2 in 6 directions
%%
%%Cr=drag(mr,mo,lo,lr)
%%where 
%%lo= [0.975 0.7 0.57 0.975/2 0.7/2 0.57/2];
%%lr=[0.92 0.655 0.538 0.92/2 0.655/2 0.538/2];
%%mr=113.2; mo=102;
%%-----------------------------------------------------------
%%Description:
%%lo- length (in m) of Oxford ROV along x,y,z and abt x,y, z axes (1 x 6).
%%lr- length (in m) of RRC ROV2   along x,y,z and abt x,y, z axes (1 x 6).
%%mr -mass in kg of RRC ROV2
%%mo- mass in kg of Oxford ROV

Co=[-340 -590 -590 -80 -100 -50];

for i=1:6
    lratio=lo(1,i)/lr(1,i);
Cr(1,i)=(mr/mo)*(lratio)^2*Co(1,i);
i-i+1;
end



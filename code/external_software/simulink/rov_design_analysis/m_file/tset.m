function [tsx,tsy,tsz,tsphi,tstheta,tsyaw] = tset(eoutd,posd,t)
%tset - SETTLING TIME OF THE TIME RESPONSE ( +/-10%)
%assume steady state value [1 0 1 0 0 1]
%[ts_x,ts_y,ts_z,ts_phi,ts_theta,ts_yaw] = tset(eoutd,t)
%--------output--------------------------
%ts_x-  x direction
%ts_y-  y direction
%ts_z-  z direction
%ts_phi-  phi direction
%ts_theta- theta direction
%ts_psi-  psi direction
%--------input--------------------------
%eoutd - position or velocity 
%posd - desired position or velocity
%t     - simulation time
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Initialization
thresx=0.10; % 10 percent - bigger than in Franklin
thresy=0.05; % 5 percent 
thresz=0.10;
thresp=0.05;
threst=0.05;
thresy=0.10;

posd=[1 0 1 0 0 1]';
 
%Calculate ts-x
n=0; 
x=eoutd(:,1);
selx=logical(abs(x-posd(1))>thresx); % +/-10%
ts_x=selx.*t;
for i=1:length(x)
    if ts_x(i) ==0 
         n=n+1;
    else
         n=0;  tsx=0; 
    end
    if n>6                     %******* trend repeats for 6 times*********
         tsx=t(i);  
         break  
    end  
end

%Calculate ts-y
n=0; 
y=eoutd(:,2);
sely=logical(abs(y-posd(2))>thresy); % +/-10%
ts_y=sely.*t;
for i=1:length(y)
    if ts_y(i) ==0 
         n=n+1;
    else
         n=0;   tsy=0; 
    end
    if n>6
         tsy=t(i); 
         break  
    end  
end


% Calculate ts-z
n=0; 
z=eoutd(:,3);
selz=logical(abs(z-posd(3))>thresz); % +/-10%
ts_z=selz.*t;
for i=1:length(z)
    if ts_z(i) ==0 
         n=n+1;
    else
         n=0;   tsz=0; 
    end
    if n>6
         tsz=t(i);
         break  
    end  
end

% Calculate ts-phi
n=0; 
phi=eoutd(:,4);
selphi=logical(abs(phi-posd(4))>thresp); % +/-10%
ts_phi=selphi.*t;
for i=1:length(z)
    if ts_phi(i) ==0 
         n=n+1;
    else
         n=0;   tsphi=0; 
    end
    if n>6
         tsphi=t(i);
         break  
    end  
end


% Calculate ts-theta
n=0; 
theta=eoutd(:,6);
seltheta=logical(abs(theta-posd(5))>threst); % +/-10%
ts_theta=seltheta.*t;
for i=1:length(z)
    if ts_theta(i) ==0 
         n=n+1;
    else
         n=0;   tstheta=0; 
    end
    if n>6
         tstheta=t(i);
         break  
    end  
end




% Calculate ts-yaw
n=0; 
yaw=eoutd(:,6);
selyaw=logical(abs(yaw-posd(6))>thresy); % +/-10%
ts_yaw=selyaw.*t;
for i=1:length(z)
    if ts_yaw(i) ==0 
         n=n+1;
    else
         n=0;   tsyaw=0; 
    end
    if n>6
         tsyaw=t(i);
         break  
    end  
end
function [mp_x,mp_y,mp_z,mp_phi,mp_theta,mp_yaw]=Mp(eoutd,posd)
%Mp - PERCENT OVERSHOOT OF THE TIME RESPONSE 
%[mp_x,mp_y,mp_z,mp_phi,mp_theta,mp_yaw] = Mp(eoutd,posd)
%--------output--------------------------
%mp_x-  x direction
%mp_y-  y direction
%mp_z-  z direction
%mp_phi-  phi direction
%mp_theta- theta direction
%mp_psi-  psi direction
%--------input--------------------------
%eoutd - position or velocity 
%posd- desired position or velocity
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Calculate Percent overshoot
format
posd=[1 0 1 0 0 1]';


x=eoutd(:,1);
if max(x(:,:))<=1
    mp_x=0;
else
  mp_x=(max(x(:,:))-posd(1)   )*100;
end


y=eoutd(:,2);
if max(y(:,:))<=1
    mp_y=0;
else
  mp_y=(max(y(:,:))-posd(2)    )*100;
end

    
z=eoutd(:,3);
if max(z(:,:))<=1
    mp_z=0;
else
  mp_z=(max(z(:,:))-posd(3)    )*100;
end

phi=eoutd(:,4);
if max(phi(:,:))<=1
    mp_phi=0;
else
  mp_phi=(max(phi(:,:))-posd(4)    )*100;
end


theta=eoutd(:,5);
if max(theta(:,:))<=1
    mp_theta=0;
else
  mp_theta=(max(theta(:,:))-posd(5)    )*100;
end

psi=eoutd(:,6);
if max(psi(:,:))<=1
    mp_yaw=0;
else
  mp_yaw=(max(psi(:,:))-posd(6)    )*100;
end

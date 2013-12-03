function [phi,theta,psi] = Rmat2euler(R)
%function [phi,theta,psi] = Rmat2euler(R)
    phi=atan(R(3,2),R(33));
    theta = -atan(R(3,1)/(sqrt(1-R(3,1).^2)));
    psi = atan2(R(2,1), R(1,1));
end

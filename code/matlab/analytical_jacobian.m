function J = analytical_jacobian(euler)
    J=zeros(6,6);
    [j,j11,j22]=eulerang(euler(1),euler(2), euler(3));
    J=j;
end
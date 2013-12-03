function mat = wln_m(J,q,eta2)
    
    W=eye(12);
    W=generateDHMatrix(q,eta2);
    
    Wi=inv(W);
    Jt=J';
    inv_prod = inv(J*Wi*Jt);
    mat=Wi*Jt*inv_prod;  
end


function mat = generateDHMatrix(q,eta2)
    
    global qmin;
    global qmax;
    dH=eye(12);
    eta2_w_factors=[4000,4000,10];
    
    %vehice orientation rates
    for i=4:6
       dh(i)=200 + eta2_w_factors(i-3)*eta2(i-3).^4 ;
    end
    
    %vehicle position rates
    for i=1:3
        dh(i)=200;
    end
    
    % link joints
    for i=1:6
        dH(i+6,i+6)= 1 + abs((((qmax(i)-qmin(i)).^2)*(2*q(i)-qmax(i)-qmin(i))) / ((4*(qmax(i)-q(i)).^2) * ((q(i)-qmin(i)).^2)));
    end
    mat=dH;
end
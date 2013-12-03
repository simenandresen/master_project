function mat_inv = mp_inv(J)
%  function mat_inv = mp_inv(J)
%  takes the pseudoinverse of the 
%  matrix J
   mat_inv = zeros(size(J,2), size(J,1));
   mat_inv = pinv(J);
end

function RGA = vrga(Gf);
%VRGA    RGA=vrga(G) returns a frequency-varying matrix (in mu-toolbox)
% 	 of the RGA of an input frequency-varying matrix G.
%
if (nargin == 0) | (nargin > 1),
   disp('usage: vrga(mat)')
   return
end
RGA = veval('.*',Gf,vpinv(vtp(Gf)));
%Alternative:
%RGA=veval('rga',Gf);



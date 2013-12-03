function [pre,post,gs]=scale(gin,meth,niter)
% SCALE  	To find diagonal scaling matrices 
%
% [pre,post,gs]=scale(g,method,iterations)
%
%	Where g is an r*c matrix to be scaled.
%
%	The default has method=2 when the algorithm finds 
%	diagonal matrices pre and post such that the largest element
%	of each row and each column of gs=post*g*pre has unit magnitude.
%
%	When method=1 the algorithm finds diagonal matrices pre and post
%	such that the sum of the elements in each row of post*g*pre is
%	equal to:-
%	         c/min(r,c)
%	and such that the sum of the elements in each column of post*g*pre
%	is equal to:-
%	         r/min(r,c)
%	Because of slow convergence the errors in the column sums can be
%	quite large. The default number of iterations is 30 but this can
%	be changed by the iterations variable.

%	
%	John M. Edmunds 11-7-97 (UMIST)
%	John M. Edmunds 7-12-91 (UMIST)
%	Copyright (c) 1991 by UMIST.
%

error(nargchk(1,3,nargin));
if(nargin<2) meth=2;end
if(nargin<3) niter=30;end
g=abs(gin);
[r,c]=size(g);
dpre=ones(1,c);
makerows=ones(1,r);
makecols=ones(1,c);
gs=gin.*dpre(makerows,:);

dpost=ones(r,1);
gs=dpost(:,makecols).*gin.*dpre(makerows,:);


%make each row sum approximately 1 or zero  and each column sum approximately 1 or zero
addrows=ones(c,1);
addcols=ones(1,r);
mindim=min([r c]);
rscale=r/mindim;
cscale=c/mindim;


% Step 1
gs=dpost(:,makecols).*g.*dpre(makerows,:);
iter=0;

rowsums=(sum(gs'))';
L=cscale*(addcols')./rowsums;
dpost=dpost.*L;


% Step 2
for iter=1:niter
  gs=dpost(:,makecols).*g.*dpre(makerows,:);

  columnsums=sum(gs);

  K=rscale*(addrows')./columnsums;
  dpre=rscale*dpre.*K;
  gs=dpost(:,makecols).*g.*dpre(makerows,:);

  rowsums=(sum(gs'))';
  L=cscale*(addcols')./rowsums;
  dpost=dpost.*L;
end
%gs=post*gin*pre;
logpre=sum(real(log(dpre)))/length(dpre);
logpost=sum(real(log(dpost)))/length(dpost);
adjust=exp((logpre-logpost)/2);

dpre=dpre/adjust;
dpost=dpost*adjust;
gs=dpost(:,makecols).*gin.*dpre(makerows,:);
post=diag(dpost);
pre=diag(dpre);

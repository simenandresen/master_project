function [pfval,pre,post,gs]=speron(gin)
% SPERON  	To find the Peron-Fobenius eigenvalues and associated
%	scaling vectors. This scaling optimises the diagonal dominance
%	ratios of the matrix.
%
%	[pfval,pre,post,gs]=speron(g)
%
%	Where g is an r*r matrix this routine calculates:-
%
%	pfval which is the largest eigenvalue of the matarix abs(g). This
%	is called the Peron Frobenius eigenvalue and if this value is
%	less than 2 the matrix can be made diagonal dominant by scaling.
%	The Peron-Fobenius scaling aims at the same dominance ratio for
%	each diagonal element, and so the algorithm will fail if any of
%	the diagonal elements are zero.
%
%	pre and post are the scalings which maximize the dominance ratios.
%
%	The variable gs returns the magnitudes of the scaled matrix
%	  post*abs(g)*pre

%	
%	copyright John M. Edmunds 11-7-97 (UMIST)
%	John M. Edmunds 1992 (UMIST)
%	Copyright (c) 1992 by UMIST.
%

error(nargchk(1,1,nargin));
g=abs(gin);
[r,c]=size(g);
gdiag=diag(g);
ze=find(gdiag==0);
% perturb any zero elements on the diagonal
if(length(ze)>0)
  pfval=1.e300;
  pre=zeros(c);
  post=zeros(r);
  gs=zeros(r,c);
  for col=ze
    disp(['Diagonal element ' int2str(col) ' was zero and so the Peron Frobenius Eigenvalue is'])
    disp('infinite, and the scaling does not exist')
  end
 return
end
t=g*inv(diag(gdiag));
[u,l]=eig(t);
[pfval,lnum]=max(diag(l));
if(nargout<2), return,end

pre=inv(diag(gdiag))*abs(diag(u(:,lnum)));
if(nargout<3), return,end
g1=g*pre;
%     to test the dominance     dom=(g1*ones(c,1))./diag(g1)
g1=g1';
gdiag=diag(g1);
t=g1*inv(diag(gdiag));
[u,l]=eig(t);
[pfval,lnum]=max(diag(l));
post=inv(diag(gdiag))*abs(diag(u(:,lnum)));
if(nargout<4), return,end
gs=post*g*pre;
%     to test the dominance     dom=(gs*ones(c,1))./diag(gs)
%     to test the dominance     dom=(ones(1,c)*gs')./diag(gs)'

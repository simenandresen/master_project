function [xpre,xpost,xgs,blocks,block]=normaliseq(xgin,method,ignore)
% NORMALISE 	permutation and scaling to normalise and diagonalise a matrix
%  [pre,post,gs,blocks,block]=normalise(g) finds permutation and scaling
%	matrices to try to normalise the matrix and make it block diagonal.
%
%	Where g is an r*c matrix
%	finds diagonal matrices pre and post
%	such that the sum of the elements in each row of post*g*pre is equal to
%	         c/min(r,c)
%	and such that the sum of the elements in each column of post*g*pre 
%	is equal to
%	         r/min(r,c)
%       attempting to make the matrix as diagonal as possible
%
%	[pre,post,gs,blocks,block]=normalise(g,method,epsilon)
%
%	When method=0  the diagonal elements tend to be ordered by magnitude.
%	When method>0  an attempt is made to bring the large off-diagonal 
%	elements nearer to the diagonal.
%	method=1  uses linkages of squares
%	method=2  uses linkages of abs This is the default action.
%
%       blocks contains pointers to begining and end of the blocks.
%	block is similar to gs but with all elements less than epsilon
%	set to zero
%	


%	John M. Edmunds 11-7-97 (UMIST)
%	John M. Edmunds 7-12-91 (UMIST)
%	Copyright (c) 1991 by UMIST.
%

error(nargchk(1,3,nargin));
if(nargin<3), ignore=0.02;end
xg=abs(xgin);
[r,c]=size(xg);
addrows=ones(c,1);
addcols=ones(1,r);
makerows=ones(r,1);
makecols=ones(c,1);
minindex=min([r c]);
[xpre,xpost,xgs]=scale(xg,1);
blocks=[1 minindex];
xgs=abs(xpost*xg*xpre);

csum=(addcols*xgs);
rsum=(xgs*addrows);
dominance=xgs;
nonzero=find(dominance);
denom=csum(makerows,:)+rsum(:,makecols);
dominance(nonzero)=dominance(nonzero)./denom(nonzero);
cswaps=[1:c];
rswaps=[1:r];
for n=1:minindex
  rd=r+1-n;
  nelem=rd*(c+1-n);
  [val,biggest]=max(reshape(dominance(rswaps(n:r),cswaps(n:c)),nelem,1));
  cbiggest=floor((biggest(1)-1)/rd)+n;
  rbiggest=rem((biggest(1)-1),rd)+n;
% swop cols of xpre to swop inputs
  cswaps([n cbiggest])=cswaps([cbiggest n]);
% swop rows of xpost to swop outputs
  rswaps([n rbiggest])=rswaps([rbiggest n]);
end
% now go back and see if we can improve the diagonal values
change=1;
while(change>0)
  change=0;
  power=1/minindex;
%  dominance=prod(diag(xgs(rswaps,cswaps)).^power);
%  pfval=speron(xgs(rswaps,cswaps))
  for n=minindex:-1 :1
    nr=rswaps(n);
    nc=cswaps(n);
    dval=xgs(nr,nc);
    dets=dval*diag(xgs(rswaps,cswaps)) - xgs(rswaps,nc).*(xgs(nr,cswaps)');
% the min value gives the best improvement)
    [detval,m]=min(dets);
% swap column m and n
    if(m~=n),
       change=change+1;
      cswaps(n)=cswaps(m);
      cswaps(m)=nc;
    end
  end
end
xpre=xpre(:,cswaps);
xpost=xpost(rswaps,:);

xgs=xpost*xg*xpre;
if(nargin<2),method=3;end
[swap,blocks]=sys_block(xgs,method,ignore);

% now sort the blocks
[rb,cb]=size(blocks);
if(rb>1)
  % now sort the blocks
  for r=1:rb
    bl=swap(blocks(r,1):blocks(r,2));
    thisblock=xgs(bl,bl);
    bswap=sys_rdr(thisblock,method);
    bl(bswap);
    swap(blocks(r,1):blocks(r,2))=bl(bswap);
  end
end

if(rb>1)
  % now re-order the blocks
  inter=zeros(rb,rb);
  for r=1:rb
    for c=1:rb
       brs=blocks(r,1);
       bre=blocks(r,2);
       bcs=blocks(c,1);
       bce=blocks(c,2);
      inter(r,c)=max(svd(xgs(swap(brs:bre),swap(bcs:bce)) ));
%      inter(c,r)=max(svd(xgs(swap(bcs:bce),swap(brs:bre)) ));
%       nelem=(bre-brs+1)*(bce-bcs+1);
%       inter(r,c)=sum(sum( xgs(swap(brs:bre),swap(bcs:bce)) )')/nelem;
%       inter(c,r)=sum(sum( xgs(swap(bcs:bce),swap(brs:bre)) )')/nelem;
    end
  end
  bswap=sys_rdr(inter,method);
  newblocks=zeros(size(blocks));
  newswap=zeros(size(swap));
  bs=1;
  for r=1:rb
% move old block bswap(r) to r 
    oldbl=blocks(bswap(r),1):blocks(bswap(r),2);
    be=bs+length(oldbl)-1;
    newblocks(r,:)=[bs be];
    newswap(bs:be)=swap(oldbl);
    bs=be+1;
  end


  [r,c]=size(xgs);
  minindex=r;
  iweights=zeros(minindex,minindex);
  for r=1:(minindex-1)
    for c=(r+1):minindex
      iweights(r,c)=(c-r)^2;
      iweights(c,r)=iweights(r,c);
    end
  end
  linkage=xgs(swap,swap);
  inertia2=sum(sum(linkage.*iweights )');


  inertia2f=zeros(rb,1);
  for r=1:rb
    interf=inter;
    for c=1:rb
      interf(r,c)=0;
      interf(c,r)=0;
    end
    interf(r,r)=1;
    bfswap=sys_rdr(interf,method);
    newblocksf=zeros(size(blocks));
    newswapf=zeros(size(swap));
    bs=1;
    for c=1:rb
% move old block bswap(c) to c 
      oldbl=blocks(bfswap(c),1):blocks(bfswap(c),2);
      be=bs+length(oldbl)-1;
      newswapf(bs:be)=swap(oldbl);
      bs=be+1;
    end
    linkage=xgs(newswapf,newswapf);
    inertia2f(r)=sum(sum(linkage.*iweights )');
  end
  [x,bf]=min(inertia2f);
  interf=inter;
  for c=1:rb
    interf(bf,c)=0;
    interf(c,bf)=0;
  end
  interf(r,r)=1;
  bfswap=sys_rdr(interf,method);
  newblocksf=zeros(size(blocks));
  newswapf=zeros(size(swap));
  bs=1;
  for c=1:rb
% move old block bfswap(r) to r 
    oldbl=blocks(bfswap(c),1):blocks(bfswap(c),2);
    be=bs+length(oldbl)-1;
    newblocksf(c,:)=[bs be];
    newswapf(bs:be)=swap(oldbl);
    bs=be+1;
  end
  swap=newswapf;
  blocks=newblocksf;
end
xpre(:,1:minindex)=xpre(:,swap);
xpost(1:minindex,:)=xpost(swap,:);


xgs=xpost*xgin*xpre;
if(method>0)
  signs=zeros(size(xpre));
  for n=1:minindex
    if(imag(xgs(n,n)))>0,  %adjust the sign for the phases
      xgs(:,n)=-xgs(:,n);
      xpre(:,n)=-xpre(:,n);
    end
  end
end
if(nargout<4), return;end
if(nargin<3), ignore=0.0;end
xx=abs(xgs);

z=find(xx<ignore);
block=xgs;
zz=zeros(length(z),1);
block(z)=zz;


%---------------------------------------------------------------------------
function [swap,blocks]=sys_block(xgin,method,ignore)
% sys_block To find the blocks of a block diagonal matrix.
%
% [swap,blocks]=sys_block(G,method,ignore)
%
%  G      The block diagonal matrix
%  method =0 or 1
%  ignore The maximum size of element to ignore
%
%  swap   The swaping matrix to get the blocks together.
%  blocks The list of blocks

%	copyright John M. Edmunds 11-7-97 (UMIST)
%	John M. Edmunds 7-12-91 (UMIST)
%	Copyright (c) 1991 by UMIST.
%

error(nargchk(1,3,nargin));
if(nargin<3), ignore=0.02;end
xg=abs(xgin);
[r,c]=size(xg);
minindex=min([r c]);

% Get them in order
swap=sys_rdr(xg,method);
xgs=xg(swap,swap);

if(method==1),
  linkage=xgs.*xgs;
else
  linkage=abs(xgs);
end
linkage=linkage+linkage';
% swop rows of xpost to swop outputs

blocks=[];
bstart=1;
% find the blocks
for n=1:minindex
  % see if we have finished a block
  if(n<minindex),
    if(max(max(linkage(bstart:n,(n+1):minindex))') <ignore),
      blocks=[blocks ; bstart n];
      bstart=n+1;
    end
  else
    blocks=[blocks ; bstart n];
  end
end


%---------------------------------------------------------------------------

function [swap]=sys_rdr(xgin,method)
%  sys_rdr To reorder a matrix to improve diagonal dominance
%
%  [swap]=sys_rdr(g,method)
%
%	Where g is an r*c matrix
%       method  =1 uses linkages of squares
%               ~=1 (default)uses linkages of abs 

%	
%	copyright John M. Edmunds 11-7-97 (UMIST)
%	John M. Edmunds 7-12-91 (UMIST)
%	Copyright (c) 1991 by UMIST.
%

error(nargchk(1,2,nargin));
xg=abs(xgin);
[r,c]=size(xg);
minindex=min([r c]);
if(nargin<2),method=3;end

% next try reorder inputs to bring large offdiagonal elements nearer to the diagonal
if(method==1),
  linkage=xg.*xg;
else
  linkage=abs(xg);
end
linkage=linkage+linkage';
% 
swap=1:minindex;

oldweights=[minindex:-1:1];
vones=ones(size(oldweights));
for n=1:minindex
  cols=[n:minindex];
  if(n>1)
%    links=vones(1:n-1)*linkage(swap(1:n-1),swap(cols));
    links=oldweights(1:n-1)*linkage(swap(1:n-1),swap(cols));
    if(n<minindex)
      slinkage=sort(linkage(swap(cols),swap(cols))-diag(diag(linkage(swap(cols),swap(cols)))));
%      linkleft=vones(cols)*linkage(swap(cols),swap(cols)) ;
      linkleft=[(minindex-n):-1:0]*slinkage;
      links=links-linkleft;
    end
  else
    slinkage=sort(linkage-diag(diag(linkage)));
    g=(minindex-1):-1:0;
    links=-g*slinkage ;
  end
  [vals,rswap]=sort(-links);
  rswap=rswap+ones(size(rswap))*(n-1);
  swap(cols)=swap(rswap);
end






npoints=10000;
nfreqs=1050;

inputlist=0:npoints;
inputlist=inputlist';
freqs=logspace(-3,1,nfreqs);




cycles=1;
times=cycles*inputlist*2*pi/npoints;
nphase=8;
integral1=zeros(nfreqs,nphase);
y=zeros(npoints+1,nphase);
nfreq=1;
for freq=freqs
  x=sin(times);
  for np=0:nphase-1
    ph=np*pi/(nphase+1);
    ti=(times*freq+ph);
    y(:,np+1)=sin(ti);
  end
  xsquared=x'*x;
  xy=x'*y;
  integral1(nfreq,:)=xy/xsquared;
  nfreq=nfreq+1;
end
integral1=abs(integral1)+0.001;
int1=max(integral1')';
loglog(freqs,int1);
hold on

cycles=10;
times=cycles*inputlist*2*pi/npoints;
nphase=8;
integral10=zeros(nfreqs,nphase);
y=zeros(npoints+1,nphase);
nfreq=1;
for freq=freqs
  x=sin(times);
  for np=0:nphase-1
    ph=np*pi/(nphase+1);
    ti=(times*freq+ph);
    y(:,np+1)=sin(ti);
  end
  xsquared=x'*x;
  xy=x'*y;
  integral10(nfreq,:)=xy/xsquared;
  nfreq=nfreq+1;
end
integral10=abs(integral10)+0.001;
int10=max(integral10')';
loglog(freqs,int10,'r');

cycles=100;
times=cycles*inputlist*2*pi/npoints;
nphase=8;
integral100=zeros(nfreqs,nphase);
y=zeros(npoints+1,nphase);
nfreq=1;
for freq=freqs
  x=sin(times);
  for np=0:nphase-1
    ph=np*pi/(nphase+1);
    ti=(times*freq+ph);
    y(:,np+1)=sin(ti);
  end
  xsquared=x'*x;
  xy=x'*y;
  integral100(nfreq,:)=xy/xsquared;
  nfreq=nfreq+1;
end
integral100=abs(integral100)+0.0001;
int100=max(integral100')';
loglog(freqs,int100,'k');
print -dmeta


  

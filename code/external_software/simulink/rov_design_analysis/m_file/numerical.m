%run rrcrov2;
%run Analy_ss;

w1=input('Enter scaling frequency (in rad/s) =');

Gw1=evalfr(sys_tot,w1);

% one-norm scaling
disp('-------------------------');
disp('One-norm scaling');
disp('-------------------------');
[pre1,post1,Gs1]=scale(Gw1,2);


% Edmund scaling (gs=post*g*pre)
disp('-------------------------');
disp('Edmunds scaling');
disp('-------------------------');
[pre1e,post1e,gs1e,blocks1e,block1e]=normalise(Gw1,1);


disp('-------------------------');
disp('Perron Frobenius scaling');
disp('-------------------------');
[pfval1p,pre1p,post1p,gs1p]=speron(Gw1);

% check whether open loop system is decoupled?
sys_tot_1=post1*sys_tot*pre1;
sys_tot_e=post1e*sys_tot*pre1e;
sys_tot_p=post1p*sys_tot*pre1p;

% choose Edmunds scaling, check row,col dominance
s=logspace(-5,2,100);
G_tot2=post1e*G_tot*pre1e;
sys_tot2=post1e*sys_tot*pre1e;
run row_col_domin

figure(7);
subplot(3,1,1);
semilogx(s,row1_ratio,'--');
hold on
semilogx(s,row2_ratio,'-');
hold on
semilogx(s,row3_ratio,':');
hold on
semilogx(s,row4_ratio,'-');
hold on
semilogx(s,row5_ratio,'-.');
hold on
semilogx(s,row6_ratio,'--');
hold on

title('Row Dominance Measure of the System (Edmunds Scaling)');
xlabel('Frequency (rad/s)');
ylabel('Gain');
legend('Row 1 ','Row 2', ' Row 3', ' Row 4', 'Row 5 ', 'Row 6 ')
drawnow



% choose one-norm scaling, check row,col dominance
s=logspace(-5,2,100);
G_tot2=post1e*G_tot*pre1e;
sys_tot2=post1*sys_tot*pre1;
run row_col_domin

subplot(3,1,2)
semilogx(s,row1_ratio,'--');
hold on
semilogx(s,row2_ratio,'-');
hold on
semilogx(s,row3_ratio,':');
hold on
semilogx(s,row4_ratio,'-');
hold on
semilogx(s,row5_ratio,'-.');
hold on
semilogx(s,row6_ratio,'--');
hold on

title('Row Dominance Measure of the System (One-norm Scaling)');
xlabel('Frequency (rad/s)');
ylabel('Gain');
drawnow


% choose Perron Frobenius scaling, check row,col dominance
s=logspace(-5,2,100);
G_tot2=post1p*G_tot*pre1p;
sys_tot2=post1p*sys_tot*pre1p;
run row_col_domin

figure(7);
subplot(3,1,3)
semilogx(s,row1_ratio,'--');
hold on
semilogx(s,row2_ratio,'-');
hold on
semilogx(s,row3_ratio,':');
hold on
semilogx(s,row4_ratio,'-');
hold on
semilogx(s,row5_ratio,'-.');
hold on
semilogx(s,row6_ratio,'--');
hold on

title('Row Dominance Measure of the System (Perron Frobenius Scaling)');
xlabel('Frequency (rad/s)');
ylabel('Gain');
drawnow
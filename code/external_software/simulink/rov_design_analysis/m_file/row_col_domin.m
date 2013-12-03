% col/ row diagonal dominance

rro=100;             % no of rows & col of the TABLE
table_domin2=cell(rro,7);
table_domin=cell(rro,7);

pp1=1;

for n=1:rro
   
   % min svd of diagonal block / max svd of off-diagonal block--> Block diagonal dominance measure
   G_sv=evalfr(G_tot2,s(n)*i);
   diag_block=[G_sv(1,1)      0         0       0            0        0;
                     0    G_sv(2,2)     0       0            0        0;
                     0         0    G_sv(3,3)   0            0        0;
                     0         0        0     G_sv(4,4)      0        0;
                     0         0        0       0        G_sv(5,5)    0;
                     0         0        0       0            0      G_sv(6,6)];
   off_diag_block=G_sv-diag_block;
   


   
   % rows dominance ratios
   sum_off_diag_row_1=sum(abs(off_diag_block(1,:)));
   sum_off_diag_row_2=sum(abs(off_diag_block(2,:)));
   sum_off_diag_row_3=sum(abs(off_diag_block(3,:)));
   sum_off_diag_row_4=sum(abs(off_diag_block(4,:)));
   sum_off_diag_row_5=sum(abs(off_diag_block(5,:)));
   sum_off_diag_row_6=sum(abs(off_diag_block(6,:)));



   row1_ratio(n)=sum_off_diag_row_1/abs(G_sv(1,1));
   row2_ratio(n)=sum_off_diag_row_2/(abs(G_sv(2,2)));
   row3_ratio(n)=sum_off_diag_row_3/abs(G_sv(3,3));
   row4_ratio(n)=sum_off_diag_row_4/(abs(G_sv(4,4)));
   row5_ratio(n)=sum_off_diag_row_5/abs(G_sv(5,5));
   row6_ratio(n)=sum_off_diag_row_6/(abs(G_sv(6,6)));

 
  
   % columns dominance ratios
   sum_off_diag_col_1=sum(abs(off_diag_block(:,1)));
   sum_off_diag_col_2=sum(abs(off_diag_block(:,2)));
   sum_off_diag_col_3=sum(abs(off_diag_block(:,3)));
   sum_off_diag_col_4=sum(abs(off_diag_block(:,4)));
   sum_off_diag_col_5=sum(abs(off_diag_block(:,5)));
   sum_off_diag_col_6=sum(abs(off_diag_block(:,6)));

   
   col1_ratio(n)=sum_off_diag_col_1/abs(G_sv(1,1));
   col2_ratio(n)=sum_off_diag_col_2/(abs(G_sv(2,2)));
   col3_ratio(n)=sum_off_diag_col_3/abs(G_sv(3,3));
   col4_ratio(n)=sum_off_diag_col_4/(abs(G_sv(4,4)));
   col5_ratio(n)=sum_off_diag_col_5/abs(G_sv(5,5));
   col6_ratio(n)=sum_off_diag_col_6/(abs(G_sv(6,6)));


  % create cell to store ratio
   table_domin{pp1,1}=s(n);         
   table_domin{pp1,2}=row1_ratio(n);  
   table_domin{pp1,3}=row2_ratio(n);
   table_domin{pp1,4}=row3_ratio(n);  
   table_domin{pp1,5}=row4_ratio(n);
   table_domin{pp1,6}=row5_ratio(n);  
   table_domin{pp1,7}=row6_ratio(n);

   
   table_domin2{pp1,1}=s(n);         
   table_domin2{pp1,2}=col1_ratio(n);  
   table_domin2{pp1,3}=col2_ratio(n);
   table_domin2{pp1,4}=col3_ratio(n);  
   table_domin2{pp1,5}=col4_ratio(n);
   table_domin2{pp1,6}=col5_ratio(n);  
   table_domin2{pp1,7}=col6_ratio(n);

   
      pp1=pp1+1;

end

clc;
format
disp('----------------------------------------------------------------')
disp('     Freq.                     row dominance ratio              ')
disp('                      (row 1)      (row 2)    (row 3)       (row 4)           (row 5)       (row 6) ')
disp('----------------------------------------------------------------')
disp(cell(table_domin));

disp('-----------------------------------------------------------------')
disp('     Freq.                       col dominance ratio              ')
disp('                       (col 1)      (col 2)          (col 3)       (col 4)       (col 5)       (col 6) ')
disp('-----------------------------------------------------------------')
disp(cell(table_domin2));


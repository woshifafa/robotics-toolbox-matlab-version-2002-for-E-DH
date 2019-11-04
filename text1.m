syms a1 a3 d3 b3 d5 b5 th1 th2 th3 th4 th5 th6 

% A(1)=[0    ,  0 ,  0 , 0  , 0  ,  0];
% A(2)=[pi/2  , 0  , a1 ,0 ,  0  ,  pi/2];
% A(3)=[-pi/2 , 0 ,  0 , 0 , d3  ,  pi/2];
% A(4)=[pi/2  , 0 ,  0 , b3 , 0  ,  0];
% A(5)=[-pi/2 , 0 ,  0,  0 , d5  ,  0];
% A(6)=[pi/2  , 0 ,  0 , b5 , 0  ,  0];

S1=s_transfer(0    ,  0 ,  0 , 0  , 0  ,  th1);
S2=s_transfer(pi/2  , 0  , a1 ,0 ,  0  ,  th2);
S3=s_transfer(-pi/2 , 0 ,  0 , 0 , d3  ,  th3);
S4=s_transfer(pi/2  , 0 ,  0 , b3 , 0  ,  th4);
S5=s_transfer(-pi/2 , 0 ,  0,  0 , d5  ,  th5);
S6=s_transfer(pi/2  , 0 ,  0 , b5 , 0  ,  th6);

S1=s_transfer(0    ,  0 ,  0 , 0  , 0  ,  th1)
S2=s_transfer(pi/2  , 0  , a1 ,0 ,  0  ,  th2)
S3=s_transfer(0   , pi/2 ,  0 , 0 , d3  ,  th3)
S4=s_transfer(0  ,  pi/2 ,  a3 , 0 , 0  ,  th4)
S5=s_transfer(-pi/2 , 0 ,  0,  0 , d5  ,  th5)
S6=s_transfer(pi/2  , 0 ,  0 , b5 , 0  ,  th6)

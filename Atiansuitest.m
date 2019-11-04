clc
clear


L{1} = link([  0         0	           0            0	      0], 'modified');
L{1}.m = 4.5201694; %LINK.m   link  mass
L{1}.r = [ 0.2,       0,        0 ]; %LINK.r 3*1 link COG vector
L{1}.I=zeros(3,3);
L{1}.Jm =  200e-6; %LINK.Jm  motor inertia
L{1}.G =  120; %LINK.G gear ratio

% L{2} = link([  pi/2      0	          -pi/2         0.5	      0], 'modified');
% L{2}.m = 10.870299;
% L{2}.r = [ -0.2075,     0.000,    -0.013992157];
% L{2}.I=zeros(3,3);
% L{2}.Jm =  200e-6;
% L{2}.G =  121;


Tiansui_arm = robot(L, 'Tian Sui NO.1 robot', 'Unimation', 'params of 8/95');

initial_pos=[0 ];

t=0:0.01:1;
qrl=[0.5 ];
[qtl,qdtl,qddtl]=jtraj(initial_pos,qrl,t);
[qtr,qdtr,qddtr]=jtraj(qrl,initial_pos,t);

qtl=[qtl;qtr;];
qdtl=[qdtl;qdtr;];
qddtl=[qddtl;qddtr;];

% figure
% plot(Tiansui_arm,qtl)

torque1=rne(Tiansui_arm,qtl,qdtl,qddtl,[0 0 9.81])

figure
plot(torque1)
xlabel('time(10ms)')
ylabel('torque(N*M)')


LL{1} = link([  0         0.5	           0             0	      0], 'standard');
LL{1}.m = 4.5201694; %LINK.m   link  mass
LL{1}.r = [ -0.5,       -0.2,        0 ]; %LINK.r 3*1 link COG vector
LL{1}.I=zeros(3,3);
LL{1}.Jm =  200e-6; %LINK.Jm  motor inertia
LL{1}.G =  120; %LINK.G gear ratio


% LL{2} = link([  0      0	          0         	      0], 'standard');
% LL{2}.m = 10.870299;
% LL{2}.r = [ -0.2075,     0.000,    -0.013992157];
% LL{2}.I=zeros(3,3);
% LL{2}.Jm =  200e-6;
% LL{2}.G =  121;


Tiansui_arm2 = robot(LL, 'Tian Sui NO.2 robot', 'Unimation', 'params of 8/95');

initial_pos=[0 ];

t=0:0.01:1;
qrl=[0.5 ];
[qtl,qdtl,qddtl]=jtraj(initial_pos,qrl,t);
[qtr,qdtr,qddtr]=jtraj(qrl,initial_pos,t);

qtl=[qtl;qtr;];
qdtl=[qdtl;qdtr;];
qddtl=[qddtl;qddtr;];

% figure
% plot(Tiansui_arm2,qtl)

torque2=rne(Tiansui_arm2,qtl,qdtl,qddtl,[0 0 9.81])

figure
plot(torque2)
xlabel('time(10ms)')
ylabel('torque(N*M)')

TO=[torque1,torque2]



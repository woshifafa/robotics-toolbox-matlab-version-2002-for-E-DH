clc
clear
% format long 
% syms a3 a4 d2 d4 d5 pi s1 s2 s3 s4 s5 s6 ox oy oz px py pz nx ny nz ax ay az real
% 
%%%%%modified D-H for suitian
a3=-0.406;
a4=-0.386;
d2=0.1485;
d4=-0.126;
dels5=-0.128;
d6=0.128;

%[alpha A dels theta D sigma offset], CONVENTION
L{1} = alink([  0         0	    0      0             0	      0], 'newmod');
L{1}.m = 4.5201694; %LINK.m   link  mass
L{1}.r = [ 0,       -0.004273996,        -0.016824473 ]; %LINK.r 3*1 link COG vector
L{1}.I=zeros(3,3);
L{1}.Jm =  200e-6; %LINK.Jm  motor inertia
L{1}.G =  120; %LINK.G gear ratio



L{2} = alink([  pi/2      0	    0      -pi/2         d2	      0], 'newmod');
L{2}.m = 10.870299;
L{2}.r = [ -0.2075,     0.000,    -0.013992157];
L{2}.I=zeros(3,3);
L{2}.Jm =  200e-6;
L{2}.G =  121;

L{3} = alink([  0         a3	      0     0             0        0], 'newmod');
L{3}.m = 3.8412581;
L{3}.r = [ -0.19670534,    0.0,      -0.12027863];
L{3}.I=zeros(3,3);
L{3}.Jm =  200e-6;
L{3}.G =  121;


L{4} = alink([ 0          a4	    0     pi/2           d4	      0], 'newmod');
L{4}.m = 1.5086689;
L{4}.r = [ 0,        0,        0.128]; %%0.013454898 +0.099174898
L{4}.I=zeros(3,3);
L{4}.Jm =  33e-6;
L{4}.G =  101;
% 
% 
L{5} = alink([  -pi/2 	 0	    dels5      0             0	      0], 'newmod');
L{5}.m = 1.5086689;
L{5}.r = [ 0,     -0.2,	    0.128  ];
L{5}.I=zeros(3,3);
L{5}.Jm =  33e-6;
L{5}.G =  101;
% 
% 
% 
L{6} = alink([  pi/2 	 0	   d6       0              0	      0], 'newmod');
% L{7} = link([ -pi/2  	 0	          pi/2           0	      0], 'modified');
L{6}.m = 0.5;
L{6}.r = [  0,        0.2,        0.10680039];
L{6}.I=zeros(3,3);
L{6}.Jm =  33e-6;
L{6}.G =  101;


L{1}.I = [  9.4481538e-3,0,0;0,9.0168561e-3,3.3216652e-4;0,3.3216652e-4,8.2272033e-3]; %LINK.I 3*3 symmetric inertia matrix
L{2}.I = [  2.2129358e-2,0,0;0,4.0782449e-1,0;0,0,4.0495181e-1];
L{3}.I = [  6.1441828e-2,0,-8.7604534e-2;0,3.2496462e-1,0;-8.7604534e-2,0,2.7007891e-1];
L{4}.I = [  1.4894609e-3,0,0;0,1.6053952e-2,-2.0786871e-3;0,-2.0786871e-3,1.6809531e-3];
% % L{5}.I = [  1.4864609e-3,0,0;0,1.2151968e-3,6.5537162e-5;0,6.5537162e-5,1.4078323e-3];
L{5}.I = [ 0.15e-3,0.1,0;0,0.15e-3,0;0,0,0.15e-3];
L{6}.I = [ 0.15e-3,0.1,0;0,0.15e-3,0;0,0,0.15e-3];


% viscous friction (motor referenced)
% L{1}.B =   1.48e-3; %LINK.B  viscous friction
% L{2}.B =   .817e-3;
% L{3}.B =    1.38e-3;
% L{4}.B =   71.2e-6;
% L{5}.B =   82.6e-6;
% L{6}.B =   36.7e-6;


% Coulomb friction (motor referenced)
% L{1}.Tc = [ .395	-.435]; %LINK.Tc  Coulomb friction
% L{2}.Tc = [ .126	-.071];
% L{3}.Tc = [ .132	-.105];
% L{4}.Tc = [ 11.2e-3 -16.9e-3];
% L{5}.Tc = [ 9.26e-3 -14.5e-3];
% L{6}.Tc = [ 3.96e-3 -10.5e-3];
Tiansui_arm = robot(L, 'Tian Sui NO.1 robot', 'Unimation', 'params of 8/95');

initial_pos=[0,-pi/2,0,pi/2,0,0];

T=fkine(Tiansui_arm,initial_pos)

% 
% figure
% plot(Tiansui_arm,initial_pos)

%initial_pos=[0,-pi/2,0,pi/2,0,0];
%initial_pos=[0.5,-1.0472,1.7453,2.4435,-0.024046,0.5];
%Tdh=fkine(Tiansui_arm,[0.5 0.5 0.5 0.5 0.5 0.5])
% figure 
% plot(Tiansui_arm,[0.5 0.5 0.5 0.5 0.5 0.5])
t=0:0.01:1;
qrl=[0.5,-1.0472,1.7453,2.4435,-0.024046,0.5];


tic
iq=qrl;
T=fkine(Tiansui_arm,qrl)
Tnew=T;
for i=1:200
  Tnew(3,4)=Tnew(3,4)+0.0005; 
  iq=ikine(Tiansui_arm,Tnew,iq);
end;
toc

[qtl,qdtl,qddtl]=jtraj(initial_pos,qrl,t);
[qtr,qdtr,qddtr]=jtraj(qrl,initial_pos,t);
% [qtk,qdtk,qddtk]=jtraj(initial_pos,qrl,t);

% qtl=[qtl;qtr;qtk];
% qdtl=[qdtl;qdtr;qdtk];
% qddtl=[qddtl;qddtr;qddtk];

qtl=[qtl;qtr;];
qdtl=[qdtl;qdtr;];
qddtl=[qddtl;qddtr;];

% Tdh=fkine(Tiansui_arm,qtl)

figure
plot(Tiansui_arm,qtl)

torque1=rne(Tiansui_arm,qtl,qdtl,qddtl,[0 0 9.81]);

figure
plot(torque1)
xlabel('time(10ms)')
ylabel('torque(N*M)')



a3=-0.406;
a4=-0.386;
d2=0.1485;
d4=0.002;
d5=0.128;


L{1} = link([  0         0	          0             0	      0], 'modified');
L{1}.m = 4.5201694; %LINK.m   link  mass
L{1}.r = [ 0,       -0.004273996,        -0.016824473 ]; %LINK.r 3*1 link COG vector
L{1}.I=zeros(3,3);
L{1}.Jm =  200e-6; %LINK.Jm  motor inertia
L{1}.G =  120; %LINK.G gear ratio



L{2} = link([  pi/2      0	         -pi/2         d2	      0], 'modified');
L{2}.m = 10.870299;
L{2}.r = [ -0.2075,     0.000,    -0.013992157];
L{2}.I=zeros(3,3);
L{2}.Jm =  200e-6;
L{2}.G =  121;

L{3} = link([  0         a3	           0             0        0], 'modified');
L{3}.m = 3.8412581;
L{3}.r = [ -0.19670534,    0.0,      -0.12027863];
L{3}.I=zeros(3,3);
L{3}.Jm =  200e-6;
L{3}.G =  121;


L{4} = link([ 0          a4	         pi/2           d4	      0], 'modified');
L{4}.m = 1.5086689;
L{4}.r = [ 0,       0 ,       0 ];%%0.013454898 0.099174898
L{4}.I=zeros(3,3);
L{4}.Jm =  33e-6;
L{4}.G =  101;
% 
% 
L{5} = link([  -pi/2 	 0	          0             d5	      0], 'modified');
L{5}.m = 1.5086689;
L{5}.r = [ 0,     -0.2,	       0];
L{5}.I=zeros(3,3);
L{5}.Jm =  33e-6;
L{5}.G =  101;
% 
% 
% 
L{6} = link([  pi/2 	 0	          0              0	      0], 'modified');
% L{7} = link([ -pi/2  	 0	          pi/2           0	      0], 'modified');
L{6}.m = 0.5;
L{6}.r = [  0,        0.2,        0.10680039];
L{6}.I=zeros(3,3);
L{6}.Jm =  33e-6;
L{6}.G =  101;


L{1}.I = [  9.4481538e-3,0,0;0,9.0168561e-3,3.3216652e-4;0,3.3216652e-4,8.2272033e-3]; %LINK.I 3*3 symmetric inertia matrix
L{2}.I = [  2.2129358e-2,0,0;0,4.0782449e-1,0;0,0,4.0495181e-1];
L{3}.I = [  6.1441828e-2,0,-8.7604534e-2;0,3.2496462e-1,0;-8.7604534e-2,0,2.7007891e-1];
L{4}.I = [  1.4894609e-3,0,0;0,1.6053952e-2,-2.0786871e-3;0,-2.0786871e-3,1.6809531e-3];
% % L{5}.I = [  1.4864609e-3,0,0;0,1.2151968e-3,6.5537162e-5;0,6.5537162e-5,1.4078323e-3];
L{5}.I = [ 0.15e-3,0.1,0;0,0.15e-3,0;0,0,0.15e-3];
L{6}.I = [ 0.15e-3,0.1,0;0,0.15e-3,0;0,0,0.15e-3];


% viscous friction (motor referenced)
% L{1}.B =   1.48e-3; %LINK.B  viscous friction
% L{2}.B =   .817e-3;
% L{3}.B =    1.38e-3;
% L{4}.B =   71.2e-6;
% L{5}.B =   82.6e-6;
% L{6}.B =   36.7e-6;


% Coulomb friction (motor referenced)
% L{1}.Tc = [ .395	-.435]; %LINK.Tc  Coulomb friction
% L{2}.Tc = [ .126	-.071];
% L{3}.Tc = [ .132	-.105];
% L{4}.Tc = [ 11.2e-3 -16.9e-3];
% L{5}.Tc = [ 9.26e-3 -14.5e-3];
% L{6}.Tc = [ 3.96e-3 -10.5e-3];
Tiansui_arm = robot(L, 'Tian Sui NO.1 robot', 'Unimation', 'params of 8/95');

initial_pos=[0,-pi/2,0,pi/2,0,0];



% figure
% plot(Tiansui_arm,initial_pos)

%initial_pos=[0,-pi/2,0,pi/2,0,0];
%initial_pos=[0.5,-1.0472,1.7453,2.4435,-0.024046,0.5];
% Tdh=fkine(Tiansui_arm,[0.5 0.5 0.5 0.5 0.5 0.5])
% figure 
% plot(Tiansui_arm,[0.5 0.5 0.5 0.5 0.5 0.5])
t=0:0.01:1;
qrl=[0.5,-1.0472,1.7453,2.4435,-0.024046,0.5];

tic
iq=qrl;
T=fkine(Tiansui_arm,qrl)
Tnew=T;
for i=1:200
  Tnew(3,4)=Tnew(3,4)+0.0005; 
  iq=ikine(Tiansui_arm,Tnew,iq);
end;
toc

[qtl,qdtl,qddtl]=jtraj(initial_pos,qrl,t);
[qtr,qdtr,qddtr]=jtraj(qrl,initial_pos,t);
% [qtk,qdtk,qddtk]=jtraj(initial_pos,qrl,t);

% qtl=[qtl;qtr;qtk];
% qdtl=[qdtl;qdtr;qdtk];
% qddtl=[qddtl;qddtr;qddtk];

qtl=[qtl;qtr;];
qdtl=[qdtl;qdtr;];
qddtl=[qddtl;qddtr;];

% Tdh=fkine(Tiansui_arm,qtl)

figure
plot(Tiansui_arm,qtl)

torque2=rne(Tiansui_arm,qtl,qdtl,qddtl,[0 0 9.81]);

figure
plot(torque2)
xlabel('time(10ms)')
ylabel('torque(N*M)')

sum(torque2-torque1)





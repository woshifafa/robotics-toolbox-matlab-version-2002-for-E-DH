clc
clear

% syms a3 a4 d2 d4 d5 pi s1 s2 s3 s4 s5 s6 ox oy oz px py pz nx ny nz ax ay az real
% 
%%%%%modified D-H for suitian
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
L{4}.r = [ 0,        0.013454898,        0.099174898];
L{4}.I=zeros(3,3);
L{4}.Jm =  33e-6;
L{4}.G =  101;


L{5} = link([  -pi/2 	 0	          0             d5	      0], 'modified');
L{5}.m = 1.5086689;
L{5}.r = [ 0,     -0.013454989,	       0.099174898];
L{5}.I=zeros(3,3);
L{5}.Jm =  33e-6;
L{5}.G =  101;



L{6} = link([  pi/2 	 0	          0              0	      0], 'modified');
% L{7} = link([ -pi/2  	 0	          pi/2           0	      0], 'modified');
L{6}.m = 0.5;
L{6}.r = [  0,        0.2,        0.10680039];
L{6}.I=zeros(3,3);
L{6}.Jm =  33e-6;
L{6}.G =  101;


% L{1}.I = [  9.4481538e-3,0,0;0,9.0168561e-3,3.3216652e-4;0,3.3216652e-4,8.2272033e-3]; %LINK.I 3*3 symmetric inertia matrix
% L{2}.I = [  2.2129358e-2,0,0;0,4.0782449e-1,0;0,0,4.0495181e-1];
% L{3}.I = [  6.1441828e-2,0,-8.7604534e-2;0,3.2496462e-1,0;-8.7604534e-2,0,2.7007891e-1];
% L{4}.I = [  1.4894609e-3,0,0;0,1.6053952e-2,-2.0786871e-3;0,-2.0786871e-3,1.6809531e-3];
% % L{5}.I = [  1.4864609e-3,0,0;0,1.2151968e-3,6.5537162e-5;0,6.5537162e-5,1.4078323e-3];
% L{5}.I = [ 0.15e-3,0,0;0,0.15e-3,0;0,0,0.15e-3];
% L{6}.I = [ 0.15e-3,0,0;0,0.15e-3,0;0,0,0.15e-3];


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



% Tiansui_arm = robot(L, 'Tian Sui NO.1 robot', 'Unimation', 'params of 8/95');
% 
% 
% 
% T=fkine(human_left_arm,[s1 s2 s3 s4 s5 s6])

% initial_pos=[0 -pi/2 0 pi/2 0 0];
% figure
% plot(Tiansui_arm,initial_pos)
% title('Tiansui__arm')



%%%%%% mixed D-H for suitian
Aa3=-0.406;
Aa4=-0.386;
Dd2=0.1485;
Dd4=-0.126;
Dd5=0.128;
Dd6=0.128;

LL{1} = link([  0         0	           0             0	      0], 'modified');
LL{2} = link([  pi/2      0	          -pi/2         Dd2	      0], 'modified');
LL{3} = link([  0         Aa3	           0             0       0], 'modified');
LL{4} = link([ 0          Aa4	         pi/2           Dd4	      0], 'modified');
LL{5} = link([  -pi/2 	 0	          0             Dd5	      0], 'standard');
LL{6} = link([  pi/2 	 0	          0             Dd6	   0], 'standard');
LL{7} = link([    0  	 0	          0             0	      0], 'standard');


LL{1}.m = 4.5201694; %LINK.m   link  mass
LL{1}.r = [ 0,       -0.004273996,        -0.016824473 ]; %LINK.r 3*1 link COG vector
LL{1}.I=zeros(3,3);
LL{1}.Jm =  200e-6; %LINK.Jm  motor inertia
LL{1}.G =  120; %LINK.G gear ratio


LL{2}.m = 10.870299;
LL{2}.r = [ -0.2075,     0.000,    -0.013992157];
LL{2}.I=zeros(3,3);
LL{2}.Jm =  200e-6;
LL{2}.G =  121;


LL{3}.m = 3.8412581;
LL{3}.r = [ -0.19670534,    0.0,      -0.12027863];
LL{3}.I=zeros(3,3);
LL{3}.Jm =  200e-6;
LL{3}.G =  121;

LL{4}.m = 1.5086689;
LL{4}.r = [ 0,        0.013454898,       0.099174898];
LL{4}.I=zeros(3,3);
LL{4}.Jm =  33e-6;
LL{4}.G =  101;



LL{5}.m = 0;
LL{5}.r = [  0,        0,        0];
LL{5}.I=zeros(3,3);
LL{5}.Jm =  0;
LL{5}.G =  0;


LL{6}.m = 1.5086689;
LL{6}.r = [ 0,         -(0.128-0.099174898), 0.013454989];
LL{6}.I=zeros(3,3);
LL{6}.Jm =  33e-6;
LL{6}.G =  101;

LL{7}.m = 0.5;
LL{7}.r = [  0,        0.2,        0.10680039];
LL{7}.I=zeros(3,3);
LL{7}.Jm =  33e-6;
LL{7}.G =  101;

% LL{1}.I = [  9.4481538e-3,0,0;0,9.0168561e-3,3.3216652e-4;0,3.3216652e-4,8.2272033e-3]; %LINK.I 3*3 symmetric inertia matrix
% LL{2}.I = [  2.2129358e-2,0,0;0,4.0782449e-1,0;0,0,4.0495181e-1];
% LL{3}.I = [  6.1441828e-2,0,-8.7604534e-2;0,3.2496462e-1,0;-8.7604534e-2,0,2.7007891e-1];
% LL{4}.I = [  1.4894609e-3,0,0;0,1.6053952e-2,-2.0786871e-3;0,-2.0786871e-3,1.6809531e-3];
% %LL{5}.I = [  1.4864609e-3,0,0;0,1.2151968e-3,6.5537162e-5;0,6.5537162e-5,1.4078323e-3];
% LL{5}.I = [ 0.15e-3,0,0;0,0.15e-3,0;0,0,0.15e-3];
% LL{6}.I = [ 0.15e-3,0,0;0,0.15e-3,0;0,0,0.15e-3];

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
Tiansui_arm_mix = robot(LL, 'Tian Sui NO.1 robot', 'Unimation', 'params of 8/95');

%initial_pos=[0,-pi/2,0,pi/2,0,0,0];
initial_pos=[0.5,-1.0472,1.7453,2.4435,0,-0.024046,0.5];
% Tmdh=fkine(Tiansui_arm_mix,[0.5 0.5 0.5 0.5 0 0.5 0.5])
% 
% figure 
% plot(Tiansui_arm_mix,[0.5 0.5 0.5 0.5 0 0.5 0.5])

t=0:0.01:1;
qrl=[0.5,-1.0472,1.7453,2.4435, 0, -0.024046,0.5];
[qtl,qdtl,qddtl]=jtraj(initial_pos,qrl,t);
[qtr,qdtr,qddtr]=jtraj(qrl,initial_pos,t);
% [qtk,qdtk,qddtk]=jtraj(initial_pos,qrl,t);

% qtl=[qtl;qtr;qtk];
% qdtl=[qdtl;qdtr;qdtk];
% qddtl=[qddtl;qddtr;qddtk];

qtl=[qtl;qtr;];
qdtl=[qdtl;qdtr;];
qddtl=[qddtl;qddtr;];

% Tmdh=fkine(Tiansui_arm_mix,qtl);

figure
plot(Tiansui_arm_mix,qtl)

torque2=rne(Tiansui_arm_mix,qtl,qdtl,qddtl,[0 0 9.81]);

figure
plot(torque2)
xlabel('time(10ms)')
ylabel('torque(N*M)')




initial_pos_mix=[0 -pi/2 0 pi/2  0 0];

figure
plot(Tiansui_arm_mix,initial_pos_mix)
title('Tiansui__arm')



% s5=0;

T=fkine(human_left_arm,[s1 s2 s3 s4 s5 s6])

jacob=jacob0(human_left_arm,[s1 s2 s3 s4 s5 s6])

TM=[nx ox ax px
    ny oy ay py
    nz oz az pz
    0   0  0  1];
X=simplify(L{2}(s2)*L{3}(s3)*L{4}(s4)*L{5}(s5)*L{6}(s6))
XL=simplify((L{1}(s1)^-1));
XKL=simplify(XL*TM)

XX=simplify(L{3}(s3)*L{4}(s4)*L{5}(s5)*L{6}(s6))
XXL=simplify((L{2}(s2)^-1)*(L{1}(s1)^-1));
XXKL=simplify(XXL*TM)

% XXT=simplify(L{2}(s2)*L{3}(s3)*L{4}(s4)*L{5}(s5))
% XXKL=simplify((L{1}(s1)^-1)*TM*(L{6}(s6)^-1))


XXX=simplify(L{4}(s4)*L{5}(s5)*L{6}(s6))
XXXL=simplify((L{3}(s3)^-1)*(L{2}(s2)^-1)*(L{1}(s1)^-1));
XXXKL=simplify(XXXL*TM)

XXXX=simplify(L{5}(s5)*L{6}(s6))
XXXXL=simplify((L{4}(s4)^-1)*(L{3}(s3)^-1)*(L{2}(s2)^-1)*(L{1}(s1)^-1));
XXXXKL=simplify(XXXL*TM)

XXXXX=simplify(L{6}(s6))
XXXXXL=simplify((L{5}(s5)^-1)*(L{4}(s4)^-1)*(L{3}(s3)^-1)*(L{2}(s2)^-1)*(L{1}(s1)^-1));
XXXXXKL=simplify(XXXL*TM)
% t1=a4*cos(s2 + s3) + a3*cos(s2) - d5*sin(s2 + s3 + s4);
% t2= a4*sin(s2 + s3) + a3*sin(s2) + d5*cos(s2 + s3 + s4);
% 
% T=simplify(t1^2+t2^2)

% t3=cos(s1)*cos(s2)*ax+cos(s2)*sin(s1)*ay+sin(s2)*az;
% t4=-cos(s1)*sin(s2)*ax-sin(s2)*sin(s1)*ay+cos(s2)*az;
% T=simplify(t3^2+t4^2)
% 
% t5=cos(s1)*cos(s2+s3)*ax+cos(s2+s3)*sin(s1)*ay+sin(s2+s3)*az;
% t6=-cos(s1)*sin(s2+s3)*ax-sin(s2+s3)*sin(s1)*ay+cos(s2+s3)*az;
% T=simplify(t5^2+t6^2)

t7=-d5*sin(s3+s4)+a4*cos(s3);
t8=d5*cos(s3+s4)+a4*sin(s3);
T=simplify(t7^2+t8^2)

% t9=cos(s6)*px-sin(s6)*py;
% t10=sin(s6)*px+cos(s6)*py+d5;
% simplify(t9^2+t10^2)

% t9=cos(s2+s3)*cos(s1)*px+cos(s2+s3)*sin(s1)*py+sin(s2+s3)*pz-a3*cos(s3);
% t10=-sin(s2+s3)*cos(s1)*px-sin(s2+s3)*sin(s1)*py+cos(s2+s3)*pz+a3*sin(s3);
% T=simplify(t9^2+t10^2)

% t11=cos(s1)*cos(s2)*px+cos(s2)*sin(s1)*py+sin(s2)*pz-a3;
% t12=-cos(s1)*sin(s2)*px-sin(s1)*sin(s2)*py+cos(s2)*pz;
% t11^2+t12^2
% T=simplify(t11^2+t12^2)


initial_pos=[0,-pi/2,0,pi/2,0,0];





% 
initial=[0 -pi/2 pi/2 pi/2 0 0 pi/2];
% test1=[0 0.4158 0.7053    0      2.093   -1    1];
% test2=[0 0.7646 0.8148 -0.7490 1.9739 0.0900 0.0330];
% 
% % figure
% % plot(human_left_arm,[1.0472,-pi/2,0+pi/2,-1.0472+pi/2,1.54,0,pi/2])
% % TT=fkine(human_left_arm,initial+test1)
% % TT(2,4)=TT(2,4)+0.10;
% % initial+test1
% % q=ikine(human_left_arm,TT,initial+test1)
% figure
% plot(human_left_arm,initial+test1)
% hold on
% plot(human_left_arm,q+[0 0 0 0.2 0 0 0])





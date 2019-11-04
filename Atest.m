clc
clear

L{1} = link([  0         0	           0            0	      0], 'modified');
L{1}.m = 4.5201694; %LINK.m   link  mass
L{1}.r = [ 0.2,       0,        0 ]; %LINK.r 3*1 link COG vector
L{1}.I=zeros(3,3);
L{1}.Jm =  200e-6; %LINK.Jm  motor inertia
L{1}.G =  120; %LINK.G gear ratio

L{2} = link([  pi/4      0.5	          -pi/2         0	      0], 'modified');
L{2}.m = 10.870299;
L{2}.r = [ -0.2075,     0.000,    -0.013992157];
L{2}.I=zeros(3,3);
L{2}.Jm =  200e-6;
L{2}.G =  121;


Tiansui_arm = robot(L, 'Tian Sui NO.1 robot', 'Unimation', 'params of 8/95');
figure
plot(Tiansui_arm,[0 -pi/2])




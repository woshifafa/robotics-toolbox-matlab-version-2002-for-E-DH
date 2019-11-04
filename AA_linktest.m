clc
clear

%[alpha A dels theta D sigma offset], CONVENTION
L{1} = alink([  0         0	    0              0             0	      0], 'newmod');
L{2} = alink([  -pi/4      0	 0.02*sqrt(2)   0      0.02*sqrt(2)	  0], 'newmod');
L{3} = alink([   pi/4        0	      0.04      pi/2           0          0], 'newmod');

Tiansui_arm = robot(L, 'Tian Sui NO.1 robot', 'Unimation', 'params of 8/95');

initial_pos=[0,0,pi/2];


plot(Tiansui_arm,initial_pos)


q0=[0 -pi/2 0 0 0 0];
q1=[0 pi/2 0 0 -pi/2 0];
tv=0:0.056:2;
qd0=[0 1.5 0 0 0.5 0];
qd1=[0 0 0 0 0 0 ];
qdd0=[0 0.2 0 0 0 0];
qdd1=[0 -1.5 0 0 0 0];
[qt,qdt,qddt] = jtraj1(q0, q1, tv, qd0, qd1,qdd0,qdd1)
 nq=find(all(qt==0)==0)
        for i=1:length(nq),
            figure('Name','PUMA560机器人仿真演示窗口---轨迹规划(关节空间）');
            subplot(3,1,1);
            plot(tv,qt(:,nq(i)));
            title('Theta');
            xlabel('Time(s)');
            ylabel(['Joint ' num2str(nq(i)) '(rad)']);
            subplot(3,1,2);
            plot(tv,qdt(:,nq(i)));
            title('Velocity');
            xlabel('Time(s)');
            ylabel(['Joint ' num2str(nq(i)) 'vel(rad/s)']);
            subplot(3,1,3);
            plot(tv,qddt(:,nq(i)));
            title('Acceleration');
            xlabel('Time(s)');
            ylabel(['Joint ' num2str(nq(i)) 'accel(rad/s^2)']);
        end
        
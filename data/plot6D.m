function [ t,pos,vel,acc ] = plot6D(s,mode)

    if nargin==1
        m = 0;
    end

    if nargin==2
        if strcmpi(mode,'diff')==1
            m = 1;
        else
            disp('unknown plot mode');
            m = 0;
        end
    end
    
    D = load(s);

    t = D(:,1);
    dt = t(2)-t(1);

    pos = D(:,2:7);
    vel = D(:,8:13);
    acc = D(:,14:19);

    figure(1)
    hold off;
    plot3(pos(:,1),pos(:,2),pos(:,3));
    hold on; grid on;
    d = max(0.1,0.01*norm(max(pos)));
    title('flange pose');
    xlabel('x'); ylabel('y'); zlabel('z');

    for i = linspace(1,size(pos,1),100);
        k = floor(i);
        R = rpy2matrix(pos(k,4:6));
        T = R*eye(3);
        plot3([pos(k,1),pos(k,1)+T(1,1)*d],...
              [pos(k,2),pos(k,2)+T(2,1)*d],...
              [pos(k,3),pos(k,3)+T(3,1)*d],'r');
          
        plot3([pos(k,1),pos(k,1)+T(1,2)*d],...
              [pos(k,2),pos(k,2)+T(2,2)*d],...
              [pos(k,3),pos(k,3)+T(3,2)*d],'g');
          
        plot3([pos(k,1),pos(k,1)+T(1,3)*d],...
              [pos(k,2),pos(k,2)+T(2,3)*d],...
              [pos(k,3),pos(k,3)+T(3,3)*d],'b');
    end
    
    pos_r = sqrt(pos(:,1).^2 + pos(:,2).^2 + pos(:,3).^2);
    vel_r = sqrt(vel(:,1).^2 + vel(:,2).^2 + vel(:,3).^2);
    acc_r = sqrt(acc(:,1).^2 + acc(:,2).^2 + acc(:,3).^2);
    
    vel_d = vel;
    acc_d = acc;

    for i=1:length(t)
        if i==1
            vel_d(i,:) = vel(i,:);
            acc_d(i,:) = acc(i,:);
        else
            vel_d(i,1:3) = (pos(i,1:3)-pos(i-1,1:3))/dt;
            acc_d(i,:) = (vel(i,:)-vel(i-1,:))/dt;
        end
    end

    vel_d_r = sqrt(vel_d(:,1).^2 + vel_d(:,2).^2 + vel_d(:,3).^2);
    acc_d_r = sqrt(acc_d(:,1).^2 + acc_d(:,2).^2 + acc_d(:,3).^2);

    figure(2)
    
    subplot(311)
    hold off;
    plot(t,pos);
    hold on;grid on;
    ylabel('pos');
    legend('x','y','z','A','B','C');
    title('ABC不连续是正常的，因为Euler角存在突变奇异点。注意Fig1中姿态是否连续');
    
    subplot(312)
    hold off;
    plot(t,vel);
    hold on;grid on;
    ylabel('vel');
    
    subplot(313)
    hold off;
    plot(t,acc);
    hold on;grid on;
    ylabel('acc');
    xlabel('time(s)');

    figure(3)
    
    subplot(311)
    hold off;
    plot(t,pos_r);
    hold on;grid on;
    ylabel('pos');
    
    subplot(312)
    hold off;
    plot(t,vel_r);
    hold on;grid on;
    ylabel('vel');
    
    subplot(313)
    hold off;
    plot(t,acc_r);
    hold on;grid on;
    ylabel('acc');
    xlabel('time(s)');

    if m==1
        figure(2)
        subplot(312)
        plot(t,vel_d,'--');
        
        subplot(313)
        plot(t,acc_d,'--');

        figure(3)
        subplot(312)
        plot(t,vel_d_r,'--');
        
        subplot(313)
        plot(t,acc_d_r,'--');
    end
end

function R = rpy2matrix(rpy)
    R = rotz(rpy(1))*roty(rpy(2))*rotx(rpy(3));
end

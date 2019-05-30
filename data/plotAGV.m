function [ t,pos,vel,acc ] = plotAGV(s,mode)

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

    pos = D(:,2:5);
    vel = D(:,6:9);
    acc = D(:,10:13);

    figure(1)
    hold off;
    plot(pos(:,1),pos(:,2));
    hold on; grid on;
    d = max(0.1,0.01*norm(max(pos(:,1:2))));
    title('red is the AGV posture, green is the wheel direction');

    for i = linspace(1,size(pos,1),100);
        k = floor(i);
        R = rpy2matrix([0,0,pos(k,3)]);
        T = R*eye(3);
        plot([pos(k,1),pos(k,1)+T(1,1)*d],...
             [pos(k,2),pos(k,2)+T(2,1)*d],'r');
         
        R = rpy2matrix([0,0,pos(k,4)]);
        T = R*eye(3);
        plot([pos(k,1),pos(k,1)+T(1,1)*d],...
             [pos(k,2),pos(k,2)+T(2,1)*d],'g');
    end
    
    pos_r = sqrt(pos(:,1).^2 + pos(:,2).^2);
    vel_r = sqrt(vel(:,1).^2 + vel(:,2).^2);
    acc_r = sqrt(acc(:,1).^2 + acc(:,2).^2);
    
    vel_d = vel;
    acc_d = acc;

    for i=1:length(t)
        if i==1
            vel_d(i,:) = vel(i,:);
            acc_d(i,:) = acc(i,:);
        else
            vel_d(i,1:2) = (pos(i,1:2)-pos(i-1,1:2))/dt;
            acc_d(i,1:2) = (vel(i,1:2)-vel(i-1,1:2))/dt;
        end
    end

    vel_d_r = sqrt(vel_d(:,1).^2 + vel_d(:,2).^2);
    acc_d_r = sqrt(acc_d(:,1).^2 + acc_d(:,2).^2);

    figure(2)
    
    subplot(311)
    hold off;
    plot(t,pos);
    hold on;grid on;
    ylabel('pos');
    legend('x','y','A','W');
    title('AW is within [-pi,pi]');
    
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
    g = rpy(1); b = rpy(2); a = rpy(3);

    R = zeros(3);

    R(1,1) = cos(a)*cos(b);
    R(1,2) = cos(a)*sin(b)*sin(g) - sin(a)*cos(g); 
    R(1,3) = cos(a)*sin(b)*cos(g) + sin(a)*sin(g);

    R(2,1) = sin(a)*cos(b);
    R(2,2) = sin(a)*sin(b)*sin(g) + cos(a)*cos(g); 
    R(2,3) = sin(a)*sin(b)*cos(g) - cos(a)*sin(g);

    R(3,1) = -sin(b);
    R(3,2) = cos(b)*sin(g);				
    R(3,3) = cos(b)*cos(g);

end

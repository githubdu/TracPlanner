function [ t,pos,vel,acc ] = plot1D(s,mode)

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

    pos = D(:,2);
    vel = D(:,3);
    acc = D(:,4);
    jer = D(:,5);
    
    vel_d = vel;
    acc_d = acc;
    jer_d = jer;

    for i=1:length(t)
        if i==1
            vel_d(i) = vel(i);
            acc_d(i) = acc(i);
            jer_d(i) = jer(i);
        else
            vel_d(i) = (pos(i)-pos(i-1))/dt;
            acc_d(i) = (vel(i)-vel(i-1))/dt;
            jer_d(i) = (acc(i)-acc(i-1))/dt;
        end
    end


    figure(1)

    subplot(411)    
    hold off;
    plot(t,pos);
    hold on;grid on;
    ylabel('p');
    
    subplot(412)
    hold off;
    plot(t,vel);
    hold on;grid on;
    ylabel('v');
    
    subplot(413)
    hold off;
    plot(t,acc);
    hold on;grid on;
    ylabel('a');
    
    subplot(414)
    hold off;
    plot(t,jer);
    hold on;grid on;
    ylabel('j');
    xlabel('time(s)');

    if m==1
        figure(1)
        subplot(412)
        plot(t,vel_d,'--');
        legend('v','dv');
        subplot(413)
        plot(t,acc_d,'--')
        legend('a','da');
        subplot(414)
        plot(t,jer_d,'--');
        legend('j','dj');
    end
end
function [ t,pos,vel] = plotOpt(s,dof)


    D = load(s);

    t = D(:,1);
    
    pos = D(:,2:7);
    vel = D(:,8:13);
    jpos = D(:,14:14+dof-1);
    jvel = D(:,14+dof:14+dof+dof-1);

    figure(1)
    hold off;
    plot3(pos(:,1),pos(:,2),pos(:,3));
    hold on; grid on;
    title('flange pose');
    d = max(0.1,0.01*norm(max(pos)));
    xlabel('x'); ylabel('y'); zlabel('z');

    for i = linspace(1,size(pos,1),100)
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
    
    figure(2)
    plot(t,jpos);
    hold on; grid on;
    title('joint angle');
    
    figure(3)
    plot(t,jvel);
    hold on; grid on;
    title('joint velocity');
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
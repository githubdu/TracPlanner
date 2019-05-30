
% 1D
plot1D('U.txt');
plot1D('T.txt');
plot1D('S.txt');
plot1D('M.txt');

% 3D
close all;
plot3D('line.txt');
plot3D('circle.txt');
plot3D('bspline.txt');
plot3D('blender.txt');

% planner6D
close all;
plot6D('pl.txt','diff');   
plot6D('pc.txt','diff');
plot6D('ps.txt','diff');
plot6D('pr.txt','diff');

% plannerOpt
plotOpt('opt.txt');

% plannerAgv
plotAGV('al.txt','diff');
plotAGV('ac.txt','diff');
plotAGV('as.txt','diff');
plotAGV('ar.txt','diff');


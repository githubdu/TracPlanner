
% µ¥Öá²å²¹
plot1D('U.txt');
plot1D('T.txt');
plot1D('S.txt');
plot1D('M.txt');

% 3D¿Õ¼ä¹ì¼£(ÎÞ×ËÌ¬)
close all;
plot3D('line.txt');
plot3D('circle.txt');
plot3D('bspline.txt');
plot3D('blender.txt');

% 6D¿Õ¼ä¹ì¼£
close all;
plot6D('pl.txt','diff');   
plot6D('pc.txt','diff');
plot6D('ps.txt','diff');
plot6D('pr.txt','diff');

% plannerOpt
plotOpt('on.txt',6);
plotOpt('or.txt',7);

% AGVÆ½Ãæ¹ì¼£
plotAGV('al.txt','diff');
plotAGV('ac.txt','diff');
plotAGV('as.txt','diff');
plotAGV('ar.txt','diff');

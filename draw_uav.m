function []=draw_uav(x,y,z)
% function draw_uav(x,y)
r=0.1;%旋翼半径
x1=x+0.2;y1=y+0.2;
x2=x+0.2;y2=y-0.2;
x3=x-0.2;y3=y+0.2;
x4=x-0.2;y4=y-0.2;
% plot([x4,x1],[y4,y1],'r','LineWidth',1);
% hold on;
% plot3(x1,y1,z,x2,y2,z,x3,y3,z,x4,y4,z);hold on;
t=0:0.01:2*pi;
X1=x1+r*cos(t);
Y1=y1+r*sin(t);
X2=x2+r*cos(t);
Y2=y2+r*sin(t);
X3=x3+r*cos(t);
Y3=y3+r*sin(t);
X4=x4+r*cos(t);
Y4=y4+r*sin(t);
plot3(X1,Y1,z,X2,Y2,z,X3,Y3,z,X4,Y4,z)
% [x2,x3],[y2,y3],'r',[x4,x1],[y4,y1],'b','LineWidth',1);
end
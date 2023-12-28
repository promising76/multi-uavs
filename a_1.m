addpath 'obstacle_avoid'
addpath 'dwa'
addpath 'creat_world'
addpath 'airplane'
addpath 'plot_graph'
addpath 'jump_local_min'  %加入逃脱局部最小值代码工作路径
start0=[-8*2.5+40 -4*2.5+40 0 0 0;%%%[x y th]
    -7*2.5+40 -7*2.5+40 0 0 0;
    0 -2*2.5+40 0 0 0;
    -4*2.5+40 -4*2.5+40 0 0 0;
    -3*2.5+40 -5*2.5+40 0 0 0;%leader
    -12*2.5+40 -10*2.5+40 0 0 0; 
    -14*2.5+40 -8*2.5+40 0 0 0;
    -5*2.5+40 -10*2.5+40 0 0 0;
    -13*2.5+40 -3*2.5+40 0 0 0;
    -15*2.5+40 -8*2.5+40 0 0 0];  
goals=[450 450 50];
obs_color = [1,0.84314,0];%障碍物颜色
obs_class = [2;1;3;2;2;1;3;3;1;1;1];%障碍物类别
obs_radius = [6;4;9;7;10;5;10;20;9;7;5]; % 障碍物半径
obs_point = [200,150,150;
            130,175,120;
            140,140,50;
            175,200,160;
           220,240,80;
            200,180,110;
            100,100,45;
            160,240,60;
            150,100,65;
            270,150,85;
            110,140,80];
creat_world(start0,goals,obs_point,obs_radius,obs_color,obs_class);%创建地图
[handles,goal_1]=formation1(goals,start0);
% goal_1= [
%         100,290,50;
%         100,280,50;
%         100,270,50;
%         100,260,50;
%         150,350,50;
%         100,250,50;
%        100,240,50;
%         100,230,50;
%         100,220,50;
%         100,210,50];
  goal2 = [
        400,290,50;
        400,280,50;
        400,270,50;
        400,260,50;
        450,350,50;
        400,250,50;
        400,240,50;
        400,230,50;
        400,220,50;
        400,210,50];
%     handles = [];
% uav_num =10;%获得无人机的数量
% for i = 1:uav_num
%     handle = drawBody(start0(i,1),start0(i,2),start0(i,3),0,0,0,[]);
%     handles = [handles,handle];
% end
data_points=multi_uav_obs_avoid(goal_1, goal2,obs_point,obs_radius,obs_class,handles);
start3=[];
data_points=data_points(end,:);
for i=1:10%飞机数量
    start3=[start3;data_points(:,(1:3)+3*(i-1))];
end
  A=zeros(10,3);
start3=[start3(:,1:2),A];
formation2(start3,goals)

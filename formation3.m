function [] = formation3 
% close all;                 % 4follower and 1 leader
countmax=50000;
goal=[300 100 50];
handles = [];
uav_num =10;%获得无人机的数量

obstacleR=0.5;% 冲突判定用的障碍物半径
%% 初始化 位置pose、速度V、加速度控制量control
start=[-8*2.5+40 -4*2.5+40 0 0 0;%%%[x y th]
    -7*2.5+40 -7*2.5+40 0 0 0;
    0 -2*2.5+40 0 0 0;
    -4*2.5+40 -4*2.5+40 0 0 0;
    -3*2.5+40 -5*2.5+40 0 0 0;%leader
    -12*2.5+40 -10*2.5+40 0 0 0; 
    -14*2.5+40 -8*2.5+40 0 0 0;
    -5*2.5+40 -10*2.5+40 0 0 0;
    -13*2.5+40 -3*2.5+40 0 0 0;
    -15*2.5+40 -8*2.5+40 0 0 0];   
    h1 = figure;
plot3(410,110,50);
% max_edge_limit=410;
% set(gca,'XLim',[0 max_edge_limit]);
% set(gca,'YLim',[0 max_edge_limit]);
% set(gca,'ZLim',[0 max_edge_limit]);
set(0,'CurrentFigure',h1);
xlabel("x");
ylabel("y");
zlabel("z");
for i = 1:uav_num
    handle = drawBody(start(i,1),start(i,2),start(i,3),0,0,0,[]);
    handles = [handles,handle];
end
 %% follower相对leader的位置
% 评价函数参数 [heading,dist,velocity,predictDT]
evalParam=[0.05,0.2,0.1,3.0];
% 模拟实验的结果
result1=[];
result2=[];
result3=[];
result4=[];
result5=[];
result6=[];
result7=[];
result8=[];
result9=[];
result10=[];
x1=start(1,:)';
x2=start(2,:)';
x3=start(3,:)';
x4=start(4,:)';
x5=start(5,:)';%x5为长机
x6=start(6,:)';
x7=start(7,:)';
x8=start(8,:)';
x9=start(9,:)';
x10=start(10,:)';
% Main loop
%  area=[-50 410 -50 410 49 51];

for m=1:countmax
      for i=1:4 
          for j=6:9
          % DWA参数输入
    distance1=sqrt((start(5,1)-start(i,1))^2+((start(5,1)-start(i,1))^2));
     distance3=sqrt((start(5,1)-start(j,1))^2+((start(5,1)-start(j,1))^2));
        if  distance1<8&&distance3<20
            Kinematic5=[40,20,8,0.2,0.01,toRadian(1)];
        else
            Kinematic5=[30,20,6,0.15,0.01,toRadian(1)];
        end
     distance2=sqrt((start(i,1)-start(i+1,1))^2+(start(i,2)-start(i+1,2))^2);
          end
      end
     if  distance2<10
        Kinematic=[50,20,10,0.4,0.01,toRadian(1)];
     else
        Kinematic=[30,20,8,0.4,0.1,toRadian(1)];
    end%若距离过远 各机速度减慢
        % 动态障碍物
    obstacle=[ -500 300 ];
    [u5,traj5]=DynamicWindowApproach(x5,Kinematic5,goal,evalParam,obstacle,obstacleR);
    if x5(1)<150
        goal1=[x5(1)-3*5 x5(2)+3*5];
        goal2=[x5(1)-6*5 x5(2)+6*5];
        goal3=[x5(1)-9*5 x5(2)+9*5];
        goal4=[x5(1)-12*5 x5(2)+12*5];%长机僚机的相对位置
        goal6=[x5(1)-3*5 x5(2)-3*5];
        goal7=[x5(1)-6*5 x5(2)-6*5];
        goal8=[x5(1)-9*5 x5(2)-9*5];
        goal9=[x5(1)-12*5 x5(2)-12*5];
        goal10=[x5(1)-15*5 x5(2)-15*5];
    else
        goal1=[x5(1)-3*3  x5(2)];
        goal2=[x5(1)-6*3 x5(2)];
        goal3=[x5(1)-6*3 x5(2)+6*3 ];
        goal4=[x5(1)-6*3 x5(2)+3*5];
        goal6=[x5(1)-6*3  x5(2)-3*3];
        goal7=[x5(1)-6*3 x5(2)-6*3];
        goal8=[x5(1)-9*3 x5(2)];
        goal9=[x5(1)-12*3 x5(2)];
        goal10=[x5(1)-15*3 x5(2)];
    end

    %% 其他飞机障碍物设置
  other_temp1=[x2(1) x2(2)  
   x3(1) x3(2)
   x4(1) x4(2)
   x6(1) x6(2)
   x7(1) x7(2)
   x8(1) x8(2)
   x9(1) x9(2)
   x10(1) x10(2)];
  other_temp2=[x1(1) x1(2)
   x3(1) x3(2)
   x4(1) x4(2)
   x6(1) x6(2)
   x7(1) x7(2)
   x8(1) x8(2)
   x9(1) x9(2)
   x10(1) x10(2)];
  other_temp3=[x2(1) x2(2)
   x1(1) x1(2)
   x4(1) x4(2)
   x6(1) x6(2)
   x7(1) x7(2)
   x8(1) x8(2)
   x9(1) x9(2)
   x10(1) x10(2)];
  other_temp4=[x2(1) x2(2)
   x3(1) x3(2)
   x1(1) x1(2)
   x6(1) x6(2)
   x7(1) x7(2)
   x8(1) x8(2)
   x9(1) x9(2)
   x10(1) x10(2)]; %将僚机作为动态障碍物进行避障
  other_temp6=[x2(1) x2(2)
   x1(1) x1(2)
   x4(1) x4(2)
   x3(1) x3(2)
   x7(1) x7(2)
   x8(1) x8(2)
   x9(1) x9(2)
   x10(1) x10(2)];
  other_temp7=[x2(1) x2(2)
   x1(1) x1(2)
   x4(1) x4(2)
   x6(1) x6(2)
   x3(1) x3(2)
   x8(1) x8(2)
   x9(1) x9(2)
   x10(1) x10(2)];
  other_temp8=[x2(1) x2(2)
   x1(1) x1(2)
   x4(1) x4(2)
   x3(1) x3(2)
   x6(1) x6(2)
   x7(1) x7(2)
   x9(1) x9(2)
   x10(1) x10(2)];
  other_temp9=[x2(1) x2(2)
   x1(1) x1(2)
   x4(1) x4(2)
   x3(1) x3(2)
   x7(1) x7(2)
   x8(1) x8(2)
   x6(1) x6(2)
   x10(1) x10(2)];
  other_temp10=[x2(1) x2(2)
   x1(1) x1(2)
   x4(1) x4(2)
   x3(1) x3(2)
   x7(1) x7(2)
   x8(1) x8(2)
   x9(1) x9(2)
   x6(1) x6(2)];
    [u4,traj4]=DynamicWindowApproach(x4,Kinematic,goal4,evalParam,other_temp4,obstacleR);
    [u3,traj3]=DynamicWindowApproach(x3,Kinematic,goal3,evalParam,other_temp3,obstacleR);
    [u2,traj2]=DynamicWindowApproach(x2,Kinematic,goal2,evalParam,other_temp2,obstacleR);
    [u1,traj1]=DynamicWindowApproach(x1,Kinematic,goal1,evalParam,other_temp1,obstacleR);
    [u10,traj10]=DynamicWindowApproach(x10,Kinematic,goal10,evalParam,other_temp10,obstacleR);
    [u9,traj9]=DynamicWindowApproach(x9,Kinematic,goal9,evalParam,other_temp9,obstacleR);
    [u8,traj8]=DynamicWindowApproach(x8,Kinematic,goal8,evalParam,other_temp8,obstacleR);
    [u7,traj7]=DynamicWindowApproach(x7,Kinematic,goal7,evalParam,other_temp7,obstacleR);
    [u6,traj6]=DynamicWindowApproach(x6,Kinematic,goal6,evalParam,other_temp6,obstacleR);
    x1 =f(x1,u1);% 机器人移动到下一个时刻
    x2 =f(x2,u2);% 机器人移动到下一个时刻
    x3 =f(x3,u3);% 机器人移动到下一个时刻
    x4 =f(x4,u4);% 机器人移动到下一个时刻
    x5 =f(x5,u5);% 机器人移动到下一个时刻
    x6 =f(x6,u6);% 机器人移动到下一个时刻
    x7 =f(x7,u7);% 机器人移动到下一个时刻
    x8 =f(x8,u8);% 机器人移动到下一个时刻
    x9 =f(x9,u9);% 机器人移动到下一个时刻
    x10 =f(x10,u10);% 机器人移动到下一个时刻
    % 模拟结果的保存 
    result1=[ result1; x1'];
    result2=[ result2; x2'];
    result3=[ result3; x3'];
    result4=[ result4; x4'];
    result5=[ result5; x5'];
    result6=[ result6; x6'];
    result7=[ result7; x7'];
    result8=[ result8; x8'];
    result9=[ result9; x9'];
    result10=[ result10; x10'];
   %====Animation====% 探索轨迹
%           area = compute_area3D(x5(1),x5(2),6);
          %% 画飞机
         result1(:,3)=50;
         result2(:,3)=50;
         result3(:,3)=50;
         result4(:,3)=50;
         result5(:,3)=50;
         result6(:,3)=50;
         result7(:,3)=50;
         result8(:,3)=50;
         result9(:,3)=50;
         result10(:,3)=50;
         org = [0,0,0];
         [angle_x,angle_y,angle_z] = xiang_liang_to_ou_la([1,1,0],org);hold on
        drawBody(x10(1),x10(2),50,angle_x,angle_y,angle_z,handles(1));
         drawBody(x9(1),x9(2),50,angle_x,angle_y,angle_z,handles(2)); 
        drawBody(x8(1),x8(2),50,angle_x,angle_y,angle_z,handles(3));
          drawBody(x7(1),x7(2),50,angle_x,angle_y,angle_z,handles(4));
          drawBody(x6(1),x6(2),50,angle_x,angle_y,angle_z,handles(5));
         drawBody(x5(1),x5(2),50,angle_x,angle_y,angle_z,handles(6));
          drawBody(x4(1),x4(2),50,angle_x,angle_y,angle_z,handles(7));
          drawBody(x3(1),x3(2),50,angle_x,angle_y,angle_z,handles(8));
          drawBody(x2(1),x2(2),50,angle_x,angle_y,angle_z,handles(9));
          drawBody(x1(1),x1(2),50,angle_x,angle_y,angle_z,handles(10));
          plot3(result10(:,1),result10(:,2),result10(:,3),'-b');hold on;%画出x的轨迹
          plot3(result9(:,1),result9(:,2),result9(:,3),'-c');hold on;%画出x的轨迹
          plot3(result8(:,1),result8(:,2),result8(:,3),'-k');hold on;%画出x的轨迹
          plot3(result7(:,1),result7(:,2),result7(:,3),'-r');hold on;%画出x的轨迹
          plot3(result6(:,1),result6(:,2),result6(:,3),'-c');hold on;%画出x的轨迹
          plot3(result5(:,1),result5(:,2),result5(:,3),'-r');hold on;%画出x的轨迹
          plot3(result4(:,1),result4(:,2),result4(:,3),'-m');hold on;%画出x的轨迹
          plot3(result3(:,1),result3(:,2),result3(:,3),'-k');hold on;%画出x的轨迹
          plot3(result2(:,1),result2(:,2),result2(:,3),'-y');hold on;%画出x的轨迹
          plot3(result1(:,1),result1(:,2),result1(:,3),'-b');hold on;%画出x的轨迹
%         grid on;
%         axis auto
     if ~isempty(traj5)
        for it=1:length(traj5(:,1))/5
            ind=1+(it-1)*5;
            traj(:,1)=traj5(ind,:);
            traj(:,2)=traj5(ind+1,:);
            traj(:,3)=50;
        end
    end%画探索轨迹
%         if norm(x5(1:2)-stop_goal(1,2)')<0.5
%             disp('Arrive Goal!!');
%         end
%     grid on;
% hold on;
%     drawnow;
%     save('x1.mat','result1')
% save('x2.mat','result2')
% save('x3.mat','result3')
% save('x4.mat','result4')
% save('x5.mat','result5')
% save('x6.mat','result6')
% save('x7.mat','result7')
% save('x8.mat','result8')
% save('x9.mat','result9')
% save('x10.mat','result10')
% save('evalParam.mat','evalParam')
if norm(x5(1:2)-goal(1,2)')<0.5
    break
end

end
function flag=hecheng2(cur_pos,obs_point)
n=length(obs_point);
center_point=sum(obs_point,1)/n;
if (cur_pos(1)-center_point(1))>0
dist=sqrt((cur_pos(1)-center_point(1)).^2+(cur_pos(2)-center_point(2)).^2);
dist_min=min(dist);
% dist
% dist_min
    if dist_min<80
        flag=1;
    else
        flag=0;
    end
else
    flag=1;
end
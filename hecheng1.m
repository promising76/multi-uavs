function flag=hecheng1(x,y,obs_point)
n=length(obs_point);
center_point=sum(obs_point,1)/n;
dist=sqrt((x-center_point(1))^2+(y-center_point(2))^2);
if dist<100
    flag=1;
else
    flag=0;
end
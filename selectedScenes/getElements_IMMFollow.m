function collections=getElements_IMMFollow(speed,distance,radius,deceleration)
%% Trajectory
segment1=[0,50,0;0,5,0];
speed1=speed(1)*ones(2,1);

angles=-pi:pi/64:pi;
angles=angles';
segment2=radius*[cos(angles)+ones(numel(angles),1) sin(angles) zeros(numel(angles),1)];
segment2=segment2+repmat([0 0 0],numel(angles),1);
speed2=speed(1)*ones(numel(angles),1);

sBreak=deceleration/2*(-speed(1)/deceleration)^2+speed(1)*(-speed(1)/deceleration);
segment3=[0,-5,0;0,-(5+sBreak),0;0,-(5+sBreak+0.8),0];
speed3=[speed(1);0.1;0.1];

%% egoVehicle
collections(1,1).classID=1;
collections(1,1).roadPoints=[segment1(1,:)+[0 distance 0];segment1;segment2;segment3];
collections(1,1).waypoints=[segment1(1,:)+[0 distance 0];segment1;segment2;segment3];
collections(1,1).speeds=[speed(1);speed1;speed2;speed3];

collections(1,1).waitTime=0*collections(1,1).speeds;
% collections(1,1).waitTime(end)=8;

%% targetVehicle
collections(1,2).classID=1;
collections(1,2).roadPoints=[segment1;segment2;segment3-[0 10 0]];
collections(1,2).waypoints=[segment1;segment2;segment3-[0 10 0]];
collections(1,2).speeds=[speed1;speed2;speed3];
collections(1,2).waitTime=0*collections(1,2).speeds;
% collections(1,2).waitTime(end)=8;
end


function collections=getCurve(speed,distance,radius)
%% trajectory
length1=60;
segment1=[0,0,0;length1-0.1,0,0];
speed1=speed(1)*ones(2,1);

angles=-pi/2:pi/16:0;
angles=angles';
segment2=radius*[cos(angles) sin(angles) zeros(numel(angles),1)];
segment2=segment2+repmat([length1 radius 0],numel(angles),1);
speed2=speed(1)*ones(numel(angles),1);

segment3=[length1+radius,radius+0.1,0;length1+radius,radius+length1,0];
speed3=[speed(1);speed(1)];

%% egoVehicle
collections(1,1).classID=1;
collections(1,1).roadPoints=[segment1(1,:)+[-distance 0 0];segment1;segment2;segment3];
collections(1,1).waypoints=[segment1(1,:)+[-distance 0 0];segment1;segment2;segment3];
collections(1,1).speeds=[speed(1);speed1;speed2;speed3];

%% targetVehicle


collections(1,2).classID=1;
collections(1,2).roadPoints=[segment1;segment2;segment3];
collections(1,2).waypoints=[segment1;segment2;segment3];
collections(1,2).speeds=[speed1;speed2;speed3];


function collections=getIMMTester(speed)

%% egoVehicle
collections(1,1).classID=1;
collections(1,1).roadPoints=[0,0,0;50,0,0];
collections(1,1).waypoints=[0,0,0];
collections(1,1).yaw=0;

%% targetVehicle
segment1=[50,50,0;50,1,0];
speed1=speed(1)*ones(2,1);

angles=-pi:pi/8:pi;
angles=angles';
segment2=35*[cos(angles)+ones(numel(angles),1) sin(angles) zeros(numel(angles),1)];
segment2=segment2+repmat([50 0 0],numel(angles),1);
speed2=speed(1)*ones(numel(angles),1);


segment3=[50,-1,0;50,-100,0];
speed3=[speed(1);speed(2)];

collections(1,2).classID=1;
collections(1,2).roadPoints=[segment1;segment2;segment3];
collections(1,2).waypoints=[segment1;segment2;segment3];
collections(1,2).speeds=[speed1;speed2;speed3];


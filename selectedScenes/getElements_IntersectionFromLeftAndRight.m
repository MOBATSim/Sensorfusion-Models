function collections=getIntersectionFromLeftAndRight(speed)
%% egoVehicle
collections(1,1).classID=1;
collections(1,1).roadPoints=[0,0,0;45,0,0];
collections(1,1).waypoints=[0,0,0;45,0,0];
collections(1,1).speeds=[0.001;0];
collections(1,1).waitTime=[0;5];

%% targetVehicle from left
collections(1,2).classID=1;
collections(1,2).roadPoints=[50,100,0;50,-100,0];
collections(1,2).waypoints=[50,100,0;50,-100,0];
collections(1,2).speeds=[speed(2);speed(2)];

%% targetVehicle from right
vertDistance=4;
collections(1,3).classID=1;
collections(1,3).roadPoints=[50+vertDistance,-100,0;50+vertDistance,100,0];
collections(1,3).waypoints=[50+vertDistance,-100,0;50+vertDistance,100,0];
collections(1,3).speeds=[speed(2);speed(2)];

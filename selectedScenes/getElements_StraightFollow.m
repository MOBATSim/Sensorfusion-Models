function collections=getStraightFollow(speed,distance)

distanceRoad=200;

%% egoVehicle
collections(1,1).classID=1;
collections(1,1).roadPoints=[0,0,0;distanceRoad,0,0];
collections(1,1).waypoints=[0,0,0;distanceRoad,0,0];
collections(1,1).speeds=[speed(1);speed(1)];

%% targetVehicle
collections(1,2).classID=1;
collections(1,2).roadPoints=[0+distance,0,0;distanceRoad+distance,0,0];
collections(1,2).waypoints=[0+distance,0,0;distanceRoad+distance,0,0];
collections(1,2).speeds=[speed(2);speed(2)];


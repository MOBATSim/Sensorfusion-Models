function collections=getElements_vehicleFrontAndBack(speed,distance)

distance=10;
%% egoVehicle
for x=1
collections(1,1).classID=1;
collections(1,1).roadPoints=[0,0,0;250,0,0];
collections(1,1).waypoints=[0,0,0;250,0,0];
collections(1,1).speeds=[speed(1);speed(1)];
end

%% vehicle in front
for x=1
collections(1,2).classID=1;
collections(1,2).roadPoints=[distance,0,0;250+distance,0,0];
collections(1,2).waypoints=[distance,0,0;250+distance,0,0];
collections(1,2).speeds=[speed(1);speed(1)];
end

%% vehicle in back
for x=1
collections(1,3).classID=1;
collections(1,3).roadPoints=[-distance,0,0;250-distance,0,0];
collections(1,3).waypoints=[-distance,0,0;250-distance,0,0];
collections(1,3).speeds=[speed(1);speed(1)];
end

%% Vehicle from front 1
for x=1
collections(1,4).classID=1;
collections(1,4).roadPoints=[250,5,0;0,5,0];
collections(1,4).waypoints=[250,5,0;0,5,0];
collections(1,4).speeds=[speed(1);speed(1)];
end

%% Vehicle from front 2
for x=1
collections(1,5).classID=1;
collections(1,5).roadPoints=[250+distance,5,0;0,5,0];
collections(1,5).waypoints=[250+distance,5,0;0,5,0];
collections(1,5).speeds=[speed(1);speed(1)];
end

%% Vehicle from front 3
for x=1
collections(1,6).classID=1;
collections(1,6).roadPoints=[250+2*distance,5,0;0,5,0];
collections(1,6).waypoints=[250+2*distance,5,0;0,5,0];
collections(1,6).speeds=[speed(1);speed(1)];
end
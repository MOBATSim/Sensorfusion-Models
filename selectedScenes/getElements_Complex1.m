function collections=getComplex1(speed)

distance=20;
%% egoVehicle
for x=1
collections(1,1).classID=1;
collections(1,1).roadPoints=[0,0,0;250,0,0];
collections(1,1).waypoints=[0,0,0;250,0,0];
collections(1,1).speeds=[speed(1);speed(1)];
end

%% vehicle in Front then right
for x=1
radius=5;
segment1=[distance,0,0;75-radius-1,0,0];
speed1=speed(1)*ones(2,1);

angles=pi/2:-pi/32:0;
angles=angles';
segment2=radius*[cos(angles) sin(angles) zeros(numel(angles),1)];
segment2=segment2+repmat([75-radius -radius 0],numel(angles),1);
speed2=speed(1)*ones(numel(angles),1);

segment3=[75,-radius-1,0;75,-radius-100,0];
speed3=[speed(1);0];

collections(1,2).classID=1;
collections(1,2).roadPoints=[segment1;segment2;segment3];
collections(1,2).waypoints=[segment1;segment2;segment3];
collections(1,2).speeds=[speed1;speed2;speed3];
collections(1,2).waitTime=0*collections(1,2).speeds;
collections(1,2).waitTime(end)=20;
end

%% vehicle in back
for x=1
collections(1,3).classID=1;
collections(1,3).roadPoints=[-distance,0,0;250-distance,0,0];
collections(1,3).waypoints=[-distance,0,0;250-distance,0,0];
collections(1,3).speeds=[speed(1);speed(1)];
end

%% vehicle from right then in front then left
for x=1
radius=5;
segment1=[75,-50,0;75,-radius-1,0];
speed1=speed(1)*ones(2,1);

angles=pi:-pi/32:pi/2;
angles=angles';
segment2=radius*[cos(angles) sin(angles) zeros(numel(angles),1)];
segment2=segment2+repmat([75+radius -radius 0],numel(angles),1);
speed2=speed(1)*ones(numel(angles),1);

segment3=[75+radius+1,0 0;150-radius-1,0,0];
speed3=[speed(1);speed(1)];

angles=-pi/2:pi/32:0;
angles=angles';
segment4=radius*[cos(angles) sin(angles) zeros(numel(angles),1)];
segment4=segment4+repmat([150-radius radius 0],numel(angles),1);
speed4=speed(1)*ones(numel(angles),1);

segment5=[150,radius+1,0;150,100,0];
speed5=[speed(1);0];

collections(1,4).classID=1;
collections(1,4).roadPoints=[segment1;segment2;segment3;segment4;segment5];
collections(1,4).waypoints=[segment1;segment2;segment3;segment4;segment5];
collections(1,4).speeds=[speed1;speed2;speed3;speed4;speed5];
collections(1,4).waitTime=0*collections(1,4).speeds;
collections(1,4).waitTime(end)=20;
end

%% Vehicle from front 1
for x=1
collections(1,5).classID=1;
collections(1,5).roadPoints=[250,5,0;0,5,0];
collections(1,5).waypoints=[250,5,0;0,5,0];
collections(1,5).speeds=[speed(1);speed(1)];
end

%% Vehicle from front 2
for x=1
collections(1,6).classID=1;
collections(1,6).roadPoints=[250+3*distance,5,0;0,5,0];
collections(1,6).waypoints=[250+3*distance,5,0;0,5,0];
collections(1,6).speeds=[speed(1);speed(1)];
end

%% Vehicle from front 3
for x=1
collections(1,7).classID=1;
collections(1,7).roadPoints=[250+6*distance,5,0;0,5,0];
collections(1,7).waypoints=[250+6*distance,5,0;0,5,0];
collections(1,7).speeds=[speed(1);speed(1)];
end
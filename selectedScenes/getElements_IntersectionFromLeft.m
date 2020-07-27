function collections=getElements_IntersectionFromLeft(speed,deceleration)
%% egoVehicle
collections(1,1).classID=1;
collections(1,1).roadPoints=[0,0,0;100,0,0];
collections(1,1).waypoints=[0,0,0;100,0,0];
collections(1,1).speeds=[speed(1);speed(1)];

%% targetVehicle
tBreak=-speed(2)/deceleration;
sBreak=deceleration/2*tBreak^2+speed(2)*tBreak;

collections(1,2).classID=1;
collections(1,2).roadPoints=[75,100,0;75,-50,0];
collections(1,2).waypoints=[75,100,0;75,5+sBreak,0;75,5,0];
collections(1,2).speeds=[speed(2);speed(2);0];
collections(1,2).waitTime=[0;0;2];


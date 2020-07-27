function collections=getDecelerationStraight(speed,distance,deceleration)
%% breakDistance
tBreak=-speed(2)/deceleration;
sBreak=deceleration/2*tBreak^2+speed(2)*tBreak;


%% egoVehicle
collections(1,1).classID=1;
collections(1,1).roadPoints=[0,0,0;100,0,0;100+sBreak,0,0];
collections(1,1).waypoints=[0,0,0;100,0,0;100+sBreak,0,0];
collections(1,1).speeds=[speed(1);speed(1);0];

%% targetVehicle
collections(1,2).classID=1;
collections(1,2).roadPoints=[0+distance,0,0;100+distance,0,0;100+distance+sBreak,0,0];
collections(1,2).waypoints=[0+distance,0,0;100+distance,0,0;100+distance+sBreak,0,0];
collections(1,2).speeds=[speed(2);speed(2);0];


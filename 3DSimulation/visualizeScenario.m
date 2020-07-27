function visualizeScenario(numMethods,result,numActors,numMethods2Show,cameraOnActorNr)
cd('3DSimulation')
% open_system('cleanModel')
load_system('cleanModel')
save_system('cleanModel', 'recentModel')
%% add Vehicles
for i=1
    add_block('lib/Vehicle','recentModel/ego');
end

for i=2:numActors+1
    add_block('lib/Vehicle',['recentModel/actor' num2str(i)]);
end

for method=numMethods
    if any(numMethods2Show==method)
        oneRow=result{method,1};
        oneRow=oneRow{1,:};
        numTracks=(size(oneRow,2))/4-numActors-1;
        for numTrack=1:numTracks
            add_block('lib/Vehicle',['recentModel/m' num2str(method) '_' num2str(numTrack)]);
        end
    end
end

save_system('recentModel', 'recentModel', 'BreakUserLinks',true,'OverwriteIfChangedOnDisk',true)


%% set all params
% ego
for i=1
    ego.x=result{numMethods(1),1}{:,1};
    ego.y=result{numMethods(1),1}{:,2};
    ego.dir=rad2deg(result{numMethods(1),1}{:,4});
    set_param('recentModel/ego/X','Table','ego.x');
    set_param('recentModel/ego/Y','Table','ego.y');
    set_param('recentModel/ego/Dir','Table','ego.dir');
    set_param('recentModel/ego','Position','[-555  -175  -520  -115]');
    set_param('recentModel/ego/Vehicle','ActorName','egoVehicle');
    set_param('recentModel/ego/Vehicle','VehColor','Red');
    add_line('recentModel','Floor/1','ego/1')
end

% actors
for i=2:numActors+1    
    actors(i).x=result{numMethods(1),1}{:,(i-1)*4+1};
    actors(i).y=result{numMethods(1),1}{:,(i-1)*4+2};
    actors(i).dir=rad2deg(result{numMethods(1),1}{:,(i-1)*4+4});
    set_param(['recentModel/actor' num2str(i) '/X'],'Table',['actors(' num2str(i) ').x']);
    set_param(['recentModel/actor' num2str(i) '/Y'],'Table',['actors(' num2str(i) ').y']);
    set_param(['recentModel/actor' num2str(i) '/Dir'],'Table',['actors(' num2str(i) ').dir']);
    thisPosition=[-390 -175 -350 -115]+(i-2)*[0 135 0 135];
    set_param(['recentModel/actor' num2str(i)],'Position',['[' num2str(thisPosition) ']']);
    set_param(['recentModel/actor' num2str(i) '/Vehicle'],'ActorName',['Actor' num2str(i)]);
    set_param(['recentModel/actor' num2str(i) '/Vehicle'],'VehColor','Green');
    
    add_line('recentModel','Floor/1',['actor' num2str(i) '/1'])
end

% tracks
for method=numMethods
    if any(numMethods2Show==method)
        switch method
            case 1
                thisColor='Blue';
            case 2
                thisColor='Black';
            case 3
                thisColor='Yellow';
            case 4
                thisColor='Orange';
        end
        oneRow=result{method,1};
        oneRow=oneRow{1,:};
        numTracks=(size(oneRow,2))/4-numActors-1;
        for numTrack=1:numTracks
            thisX=result{method,1}{:,(numActors+numTrack)*4+1};
            thisX(isnan(thisX))=0;
            thisY=result{method,1}{:,(numActors+numTrack)*4+2};
            thisY(isnan(thisY))=0;
            thisDir=rad2deg(result{method,1}{:,(numActors+numTrack)*4+4});
            thisDir(isnan(thisDir))=0;
            tracks.(['m' num2str(method)])(numTrack).x=thisX;
            tracks.(['m' num2str(method)])(numTrack).y=thisY;
            tracks.(['m' num2str(method)])(numTrack).dir=thisDir;
            
            set_param(['recentModel/m' num2str(method) '_' num2str(numTrack) '/X']...
                ,'Table',['tracks.' 'm' num2str(method) '(' num2str(numTrack) ').x']);
            set_param(['recentModel/m' num2str(method) '_' num2str(numTrack) '/Y']...
                ,'Table',['tracks.' 'm' num2str(method) '(' num2str(numTrack) ').y']);
            set_param(['recentModel/m' num2str(method) '_' num2str(numTrack) '/Dir']...
                ,'Table',['tracks.' 'm' num2str(method) '(' num2str(numTrack) ').dir']);
            
            thisPosition=[-390 -175 -350 -115]+(numTrack-1)*[0 135 0 135]+(method)*[165 0 165 0];
            set_param(['recentModel/m' num2str(method) '_' num2str(numTrack)],...
                'Position',['[' num2str(thisPosition) ']']);
            set_param(['recentModel/m' num2str(method) '_' num2str(numTrack) '/Vehicle'],...
                'ActorName',['m' num2str(method) '_' num2str(numTrack)]);
            set_param(['recentModel/m' num2str(method) '_' num2str(numTrack) '/Vehicle'],...
                'VehColor',thisColor);
            
            add_line('recentModel','Floor/1',['m' num2str(method) '_' num2str(numTrack) '/1'])
        end
    end
end

%% save 
save_system('recentModel')

%% set position of camera to Actor 2 (or others)
open_system('recentModel/Scene Configuration')
close_system('recentModel/Scene Configuration')
switch cameraOnActorNr
    case 1
        set_param('recentModel/Scene Configuration','vehTag','egoVehicle');
    otherwise
        set_param('recentModel/Scene Configuration','vehTag',['Actor' num2str(cameraOnActorNr)]);
end


assignin('base','ego',ego);
assignin('base','actors',actors);
assignin('base','tracks',tracks);

%% Slow down simulation by changing the gain-block's gain
slowDown=8;
dtOfTable=slowDown*seconds(result{numMethods(1),1}.Time(2)-result{numMethods(1),1}.Time(1));
set_param('recentModel/Gain','Gain',['1/' num2str(dtOfTable)])

%% setup start and stop time of Simulation
endTime=slowDown*seconds(result{numMethods(1),1}.Time(end,1));
sim('recentModel','StartTime','0','StopTime',num2str(endTime),'SrcWorkspace','current');
close_system('recentModel',1,'OverwriteIfChangedOnDisk',true)
cd('../')





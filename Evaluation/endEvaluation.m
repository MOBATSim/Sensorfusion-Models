%% creating a timetable (allResults) with the kinematic data of the tracks (position+velocity+yaw)
function allResults=endEvaluation(allData,confirmedTracks,scenario,sensorTime,numMethods)
allTimes=[allData.Time]';
allActorData=[allData.ActorPoses];

%% create the scenario data again, but with a better time resolution
realData=getBetterResolution(scenario,allData);

%% Store the actors' values first
allActors=timetable(seconds(realData(1).time));
for numActor=1:size(allData(1,1).ActorPoses,1)
    for numCoord=1:2
        switch numCoord
            case 1
                allActors.(['actor' num2str(numActor) 'x'])=realData(numActor).position(:,1);
            case 2
                allActors.(['actor' num2str(numActor) 'y'])=realData(numActor).position(:,2);
        end        
    end
    allActors.(['actor' num2str(numActor) 'v'])=realData(numActor).velocity;
    allActors.(['actor' num2str(numActor) 'dir'])=realData(numActor).yaw;
end

%% Afterwards every method gets their own result timetable
 for method=numMethods
     switch method
            case 1
                assignmentMethod="GNN";
                filterType="cvKF";
            case 2
                assignmentMethod="GNN";
                filterType="IMM";
            case 3
                assignmentMethod="JPDA";
                filterType="cvKF";
            case 4
                assignmentMethod="JPDA";
                filterType="IMM";
     end
     
     %% the Actor data determins the first columns
     thisReference=allActors;
     
     %% all confirmed tracks are evaluated in the timetable
     thisTracks=confirmedTracks{method,1};
     for numTrack=1:size(thisTracks,1)
         thisTrack=thisTracks(numTrack,1);
         
         %% find the timespawn of the track and take those times from the 
         % actor timetable (else there will be errors because of rounding)
         starter=thisTrack.historyPosition{1,1}(1,1);
         [~,startIdx]=min(abs(seconds(thisReference.Time)-starter));
         ender=thisTrack.historyPosition{1,1}(end,1);
         [~,endIdx]=min(abs(seconds(thisReference.Time)-ender));
         thisTimes=thisReference.Time(startIdx:sensorTime/0.01:endIdx); 
         
         %% Now all values are transfered and named correctly
%          endLength=numel([thisTrack.historyDetections(:,1)]);
%          getLast=cellfun(@isempty,thisTrack.historyDetections(:,1));
%          endLength=find(~getLast,1,'last');
         endLength=numel(thisTimes);
         thisPositions=thisTrack.historyPosition{1,1}(1:endLength,2:3);
         thisSpeeds=thisTrack.historySpeed(1:endLength,2);
         thisDirections=thisTrack.historyDirection(1:endLength,2);
         thisNames=[{['track' num2str(numTrack) 'x']} {['track' num2str(numTrack) 'y']}...
             {['track' num2str(numTrack) 'v']} {['track' num2str(numTrack) 'dir']}];
         thisTimetable=array2timetable([thisPositions thisSpeeds thisDirections]...
             ,'RowTimes',thisTimes,'variableNames',thisNames);
         
         %% The two timetables get now synchronized with the actor timetable
         thisReference=synchronize(thisReference,thisTimetable,'first','previous');         
     end         
         
     allResults{method,1}=thisReference;
 end

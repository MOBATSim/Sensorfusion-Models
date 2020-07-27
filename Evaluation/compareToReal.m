%%Looking for the best actor that fits to this track
function [numBestActor,confirmedTracks]= compareToReal(tracks,oldTracks,allData,dt,method)

%% collect all actor positions in one array for each
for numActor=1:size(allData(1).ActorPoses,1)
    realPositions=[];
    for timeFrame=1:size(allData,2)
        thisTime=allData(1,timeFrame).Time;
        thisRealPosition=allData(timeFrame).ActorPoses(numActor,1).Position;
        realPositions=[realPositions; thisTime thisRealPosition(1:2)];
    end
    realActorPositions{1,numActor}=realPositions;
end

%% detect and collect all tracks that were confirmed
confirmedTracks=[];
for numTrack=1:numel(tracks)+numel(oldTracks)
    if (numTrack<=numel(tracks))
        thisTrack=tracks(numTrack,1);
    else
        thisTrack=oldTracks(numTrack-numel(tracks));
    end
    
    if (thisTrack.confirmed)
        if isempty(confirmedTracks)
            confirmedTracks=thisTrack;
        else
            confirmedTracks(end+1,1)=thisTrack;
        end
    end
end
%% Compare the track's position over time with every actor
numBestActor=[];
for numTrack=1:size(confirmedTracks,1)
    thisTrack=confirmedTracks(numTrack,1);
    allDetectedPositions=thisTrack.historyPosition{1,1};
    startFrame=thisTrack.historyPosition{1,1}(1,1);
    endFrame=thisTrack.historyPosition{1,1}(end,1);
    allDifferences=[];
    
    for numActor=1:size(allData(1).ActorPoses,1)
        realPositions=[];
        [~,startIdx]=min(abs([allData.Time]-startFrame));
        [~,endIdx]=min(abs([allData.Time]-endFrame));
        for idx=startIdx:endIdx
            realPositions=[realPositions; realActorPositions{1,numActor}(idx,2:3)];
        end
        allPositions=[allDetectedPositions realPositions];
        differencePosition=sqrt((allPositions(:,2)-allPositions(:,4)).^2+(allPositions(:,3)-allPositions(:,5)).^2);
        allDifferences(numActor,1)=sqrt(mean(differencePosition.^2));
        allDifferenceValues{numTrack,numActor}=differencePosition;
    end
    allDifferences(1)=100000;
    %% Find the best Actor
    numBestActor(numTrack,1)=find(allDifferences==min(allDifferences));
    numBestActor(numTrack,2)=min(allDifferences);
end

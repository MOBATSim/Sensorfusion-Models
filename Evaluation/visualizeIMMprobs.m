%% Visualizing the interactive multiple model filter's model probability over time
function visualizeIMMprobs(confirmedTracks,numMethods, actorNrIn,allbestActors)
%% New figure
allTitles=[{'GNN IMM'} {'JPDA IMM'}];

%% Plotting the probabilities for every method for every filter
for i=numMethods
    if i==2 || i==4
        if ~isempty(confirmedTracks{i,1})
            thisTracks=confirmedTracks{i,1};
            [trackNumIn,~]=find(allbestActors{i,1}(:,1)==actorNrIn);
            for trackNum=trackNumIn'
                
                thisTimes=seconds(thisTracks(trackNum).historyPosition{1,1}(:,1));
                
                figure();
                hold on;
                grid on;
                xlabel("Time");
                ylabel("Model probabilities");
                title([allTitles{i/2} ' Track Nr. ' num2str(trackNum)])
                plot(thisTimes(2:1+size(thisTracks(trackNum).historyIMM,1)),thisTracks(trackNum).historyIMM(:,1))
                plot(thisTimes(2:1+size(thisTracks(trackNum).historyIMM,1)),thisTracks(trackNum).historyIMM(:,2))
                plot(thisTimes(2:1+size(thisTracks(trackNum).historyIMM,1)),thisTracks(trackNum).historyIMM(:,3))
                
                legend(["constant velocity" "constant acceleration" "constant turnrate"],'Location','best')
            end
        end
    end
end


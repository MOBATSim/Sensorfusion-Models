%% Visualizing multiple criteria of the different methods compared to the real values
function allDifferences=visualizeDifference(result,numMethods,allbestActors,numActors,sensorFrequency,confirmedTracks)
sensorFrequency=round(sensorFrequency,2);
allDifferences=[];
%% for every actor a own figure window is opened
for numActor=2:numActors+1
    f=figure();%9+numActor);
    f.Name=join(["Actor nr." num2str(numActor)]);
    f.NumberTitle='off';
    clf
    allLegends=[];
    
    %% The different properties are drawn within 4 subplots
    thisResult=result{numMethods(1),1};
    
%     %% Subplot1 : a X-Y coordinate system with the global trajectory of the actor
%     subplot(2,2,1)
%     plot(-1*thisResult{:,(numActor-1)*4+2},thisResult{:,(numActor-1)*4+1},'g')
%     hold on
%     axis equal
%     title("Map")
    
    %% Subplot 2 : a distance over time plot with the nearest distance between the ego vehicle's CENTER
    % and the nearest point of the actor
    subplot(2,2,1)
    hold on
    grid on;
    xlabel("Time")
    ylabel("Difference in range [m]")
    actorYaw=wrapTo2Pi(thisResult{:,(numActor-1)*4+4});
    actorX=thisResult{:,(numActor-1)*4+1};
    actorY=thisResult{:,(numActor-1)*4+2};
    % getting the nearest edge point
    dx2Ego=thisResult{:,1}-actorX;
    dy2Ego=thisResult{:,2}-actorY;
    angle2Ego=wrapTo2Pi(atan2(dy2Ego,dx2Ego));
    angles=unwrap([actorYaw angle2Ego]);
    diffAngle=wrapTo2Pi(angles(:,2)-angles(:,1));
    for k=1:numel(diffAngle)
        if diffAngle(k)<pi/2
            offsetX=3.7;
            offsetY=0.9;
        elseif diffAngle(k)==pi/2
            offsetX=0;
            offsetY=0.9;
        elseif diffAngle(k)<pi
            offsetX=-1;
            offsetY=0.9;
        elseif diffAngle(k)==pi
            offsetX=-1;
            offsetY=0;
        elseif diffAngle(k)<3/2*pi
            offsetX=-1;
            offsetY=-0.9;
        elseif diffAngle(k)==3/2*pi
            offsetX=0;
            offsetY=-0.9;
        elseif diffAngle(k)<2*pi
            offsetX=3.7;
            offsetY=-0.9;
        elseif diffAngle(k)==2*pi
            offsetX=3.7;
            offsetY=0;
        end
        r=rotationVectorToMatrix([0 0 -actorYaw(k)]);
        thisOffset=r(1:2,1:2)*[offsetX;offsetY];
        actorX(k)=actorX(k)+thisOffset(1);
        actorY(k)=actorY(k)+thisOffset(2);
    end    
    distEgo2ActorEdge=vecnorm([thisResult{:,1}-actorX thisResult{:,2}-actorY],2,2);
    angleEgo2ActorEdge=atan2(thisResult{:,2}-actorY,thisResult{:,1}-actorX);
%     plot(thisResult.Time,distEgo2ActorEdge,'g')
    title(['Actor Nr.' num2str(numActor) ' Range'])
    %% Init subplot 2
    subplot(2,2,2)
    hold on
    grid on;
    xlabel("Time")
    ylabel(['Difference in direction' newline 'to the actor edge [rad]'])
    %% Subplot3 : a velocity over time plot
    subplot(2,2,3)
    thisActorVel=thisResult{:,(numActor-1)*4+3};
    plot(thisResult.Time,thisActorVel,'g')
    hold on
    grid on;
    title(['Actor Nr.' num2str(numActor) ' Velocity'])
    xlabel("Time")
    ylabel("Velocity [m/s]")
    
    %% Subplot4 : a direction over time plot
    subplot(2,2,4)
    thisActorAngles=unwrap(thisResult{:,(numActor-1)*4+4});
    plot(thisResult.Time,thisActorAngles,'g')
    hold on
    grid on;
    title(['Actor Nr.' num2str(numActor) ' Velocity direction'])
    xlabel("Time")
    ylabel(['Direction' newline 'of the velocity [rad]'])
    
    %% adding the respective entry to the legend array
%     allLegends=[allLegends string(['Actor nr ' num2str(numActor)])];
    
    %% Add the different tracks of the different methods to the subplots
    % using their respective color
    for i=numMethods
        switch i
            case 1
                thisColor='b';
            case 2
                thisColor='k';
            case 3
                thisColor='y';
            case 4
                thisColor='#D95319';
        end
        
        %% Take the track<->actor assignment from the compareToReal- function
        [suitTracks,~]=find(allbestActors{i,1}(:,1)==numActor);
        if ~isempty(suitTracks)
            for numTrack=suitTracks'
                %% Take every time that suits to this actor
                thisResult=result{i,1};
                
                %% getting birth and death "dates" to only plot the track in 
                % his respective timewindow where he was active
                thisAge=confirmedTracks{i,1}(numTrack,1).age;
                thisBirth=find(~isnan(thisResult{:,(numActors+numTrack)*4+1}),1,"first");
                thisBirth=(thisBirth-1)/(sensorFrequency/0.01)+1;
                thisDeath=thisBirth+thisAge-1;
                thisBirth=thisBirth+3;
                
%                 %% plot trajectory
%                 subplot(2,2,1)
%                 plot(-1*thisResult{1:end,(numActors+numTrack)*4+2},...
%                     thisResult{1:end,(numActors+numTrack)*4+1},'Color',thisColor);
%                 axis equal
                
                %% plot distance between track and ego CENTER
                subplot(2,2,1)
%                 actorYaw=wrapTo2Pi(thisResult{:,(numActor-1)*4+4});
%                 actorX=thisResult{:,(numActor-1)*4+1};
%                 actorY=thisResult{:,(numActor-1)*4+2};
%                 % get nearest edge point of actor
%                 dx2Ego=thisResult{:,1}-actorX;
%                 dy2Ego=thisResult{:,2}-actorY;
%                 angle2Ego=wrapTo2Pi(atan2(dy2Ego,dx2Ego));
%                 angles=unwrap([actorYaw angle2Ego]);
%                 diffAngle=wrapTo2Pi(angles(:,2)-angles(:,1));
%                 for k=1:numel(diffAngle)
%                     if diffAngle(k)<pi/2
%                         offsetX=3.7;
%                         offsetY=0.9;
%                     elseif diffAngle==pi/2
%                         offsetX=0;
%                         offsetY=0.9;    
%                     elseif diffAngle<pi
%                         offsetX=-1;
%                         offsetY=0.9;
%                     elseif diffAngle==pi
%                         offsetX=-1;
%                         offsetY=0; 
%                     elseif diffAngle<3/2*pi
%                         offsetX=-1;
%                         offsetY=-0.9;
%                     elseif diffAngle==3/2*pi
%                         offsetX=0;
%                         offsetY=-0.9;
%                     elseif diffAngle<2*pi
%                         offsetX=3.7;
%                         offsetY=-0.9;
%                     elseif diffAngle==2*pi
%                         offsetX=3.7;
%                         offsetY=0;    
%                     end
%                     r=rotationVectorToMatrix([0 0 -actorYaw(k)]);
%                     thisOffset=r(1:2,1:2)*[offsetX;offsetY];
%                     actorX(k)=actorX(k)+thisOffset(1);
%                     actorY(k)=actorY(k)+thisOffset(2);
%                 end
%                 
%                 dX=thisResult{:,(numActors+numTrack)*4+1}-actorX;
%                 dY=thisResult{:,(numActors+numTrack)*4+2}-actorY;
%                 dist2EgoActorEdge=vecnorm([thisResult{:,1}-actorX thisResult{:,2}-actorY],2,2);
                dist2EgoTrack=vecnorm([thisResult{:,1}-thisResult{:,(numActors+numTrack)*4+1}...
                    thisResult{:,2}-thisResult{:,(numActors+numTrack)*4+2}],2,2);
                angleEgo2Track=atan2(thisResult{:,2}-thisResult{:,(numActors+numTrack)*4+2},thisResult{:,1}-thisResult{:,(numActors+numTrack)*4+1});
                
                %% Plotting the difference between the optimal and the estimated distance
                thisTimes=thisResult.Time(1:sensorFrequency/0.01:end);
                thisDifference=distEgo2ActorEdge-dist2EgoTrack;
                thisPlotDiff=thisDifference(1:sensorFrequency/0.01:end);
                thisDeath=min(thisDeath,numel(thisPlotDiff));
                allDifferences.Range{numActor,i,numTrack}=thisPlotDiff(thisBirth:thisDeath);
                plot(thisTimes(thisBirth:thisDeath),thisPlotDiff(thisBirth:thisDeath),'Color',thisColor)
                
                %% Plotting the actual distance between ego and track
                thisDistance=dist2EgoTrack(1:sensorFrequency/0.01:end);
%                 plot(thisTimes(thisBirth:thisDeath),thisDistance(thisBirth:thisDeath),'Color',thisColor)
                
                %% Plot angle difference of range
                subplot(2,2,2)
                hold on
                title(['Actor Nr.' num2str(numActor) ' Range direction'])
                thisAngleDifference=unwrap(angleEgo2Track-angleEgo2ActorEdge);
                thisAnglePlotDiff=thisAngleDifference(1:sensorFrequency/0.01:end);
                thisAnglePlotDiff=thisAnglePlotDiff-2*pi*round(mean(thisAnglePlotDiff(thisBirth:thisDeath))/2/pi);
                allDifferences.RangeAngles{numActor,i,numTrack}=thisAnglePlotDiff(thisBirth:thisDeath);
                plot(thisTimes(thisBirth:thisDeath),thisAnglePlotDiff(thisBirth:thisDeath),'Color',thisColor)
                
                allDifferences.BirthDeath{numActor,i,numTrack}=[thisBirth,thisDeath];
                
                %% plotting the velocity of the track over time
                subplot(2,2,3)
                thisTimes=thisResult.Time;
                thisVel=thisResult{:,(numActors+numTrack)*4+3};
                allDifferences.Velocity{numActor,i,numTrack}=thisVel(1+int64((thisBirth-1)*sensorFrequency/0.01):1+int64((thisDeath-1)*sensorFrequency/0.01))...
                    -thisActorVel(1+int64((thisBirth-1)*sensorFrequency/0.01):1+int64((thisDeath-1)*sensorFrequency/0.01));
                allDifferences.Velocity{numActor,i,numTrack}=allDifferences.Velocity{numActor,i,numTrack}(1:sensorFrequency/0.01:end,1);
                plot(thisTimes(1+int64((thisBirth-1)*sensorFrequency/0.01):1+int64((thisDeath-1)*sensorFrequency/0.01)),...
                    thisVel(1+int64((thisBirth-1)*sensorFrequency/0.01):1+int64((thisDeath-1)*sensorFrequency/0.01)),'Color',thisColor)
                
                %% plotting the direction of the track
                subplot(2,2,4)
                thisAngles=unwrap(thisResult{:,(numActors+numTrack)*4+4});
                allDifferences.VelocityAngles{numActor,i,numTrack}=thisAngles(1+int64((thisBirth-1)*sensorFrequency/0.01):1+int64((thisDeath-1)*sensorFrequency/0.01))...
                    -thisActorAngles(1+int64((thisBirth-1)*sensorFrequency/0.01):1+int64((thisDeath-1)*sensorFrequency/0.01));
                thisAngles=thisAngles-2*pi*round(mean(allDifferences.VelocityAngles{numActor,i,numTrack})/2/pi);
                allDifferences.VelocityAngles{numActor,i,numTrack}=allDifferences.VelocityAngles{numActor,i,numTrack}-2*pi*round(mean(allDifferences.VelocityAngles{numActor,i,numTrack})/2/pi);
                allDifferences.VelocityAngles{numActor,i,numTrack}=allDifferences.VelocityAngles{numActor,i,numTrack}(1:sensorFrequency/0.01:end,1);
                
                plot(thisTimes(1+int64((thisBirth-1)*sensorFrequency/0.01):1+int64((thisDeath-1)*sensorFrequency/0.01)),...
                    thisAngles(1+int64((thisBirth-1)*sensorFrequency/0.01):1+int64((thisDeath-1)*sensorFrequency/0.01)),'Color',thisColor)
                
                switch i
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
                %% Add legend entrys
                allLegends=[allLegends join([assignmentMethod,filterType])];
            end
        end
    end

    %% Add the legends to the first|(every subplot)
    for i=1%:4        
        subplot(2,2,i)        
            legend(allLegends,'location','best')
    end
end




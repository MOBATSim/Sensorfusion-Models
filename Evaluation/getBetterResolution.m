%% resimulate the scenario to get a better time resolution in the actor data
function realData=getBetterResolution(scenario,allData)
%% find simulation end time
endTime=[allData.Time];
endTime=endTime(end);

%% resample with sample time 0.01s
scenario.SampleTime=0.01;
restart(scenario)

%% reallocate data array
realData(size(scenario.Actors,2)).time=[];
realData(size(scenario.Actors,2)).position=[];
realData(size(scenario.Actors,2)).yaw=[];
realData(size(scenario.Actors,2)).velocity=[];

%% new simulation
while scenario.SimulationTime-endTime<=0.001
    %% Storing all important variables
    for i=1:size(scenario.Actors,2)
        realData(i).position=[realData(i).position; scenario.Actors(1,i).Position];
        realData(i).yaw=[realData(i).yaw; atan2(scenario.Actors(1,i).Velocity(2),scenario.Actors(1,i).Velocity(1))];
        realData(i).velocity=[realData(i).velocity; norm(scenario.Actors(1,i).Velocity)];
        realData(i).time=[realData(i).time; scenario.SimulationTime];
    end
    %% Then advancing to the next time step
    advance(scenario);
end
restart(scenario)
        
        
%% Plotting all trajectories of all actors and of all tracks of the respective method
function visualizeMap(result,numMethods,numActors)
%% new figure
f=figure();
f.Name="Global map";
f.NumberTitle='off';

axis equal
hold on
%% 1 Subplot per method
methodCount=numel(numMethods);
switch methodCount
    case 1
        numRows=1;
        numCols=1;
    otherwise
        numRows=round(methodCount/2);
        numCols=2;
end

%% Plotting all trajectories
n=1;
for i=numMethods
    thisResult=result{i,1};
    switch i
        case 1
            assignmentMethod='GNN';
            filterType='cvKF';
            thisColor='b';
        case 2
            assignmentMethod='GNN';
            filterType='IMM';
            thisColor='k';
        case 3
            assignmentMethod='JPDA';
            filterType='cvKF';
            thisColor='y';
        case 4
            assignmentMethod='JPDA';
            filterType='IMM';
            thisColor='#D95319';
    end
    
    subplot(numRows,numCols,n)
    hold on
    for j=1:size(thisResult,2)/4
        if j>numActors+1
            plot(-1*thisResult{:,(j-1)*4+2},thisResult{:,(j-1)*4+1},'Color',thisColor)
        else
            plot(-1*thisResult{:,(j-1)*4+2},thisResult{:,(j-1)*4+1},'g')
        end
    end
    axis equal
    xlabel("-1*Y[m]")
    ylabel("X[m]")
    
    title([assignmentMethod ' - ' filterType])
    
    n=n+1;
end


function [allErrors,allMeans,allStd]=visualizeBoxPlots(allDifferences,numActorIn)
if numActorIn>1 && numActorIn <= size(allDifferences.Range,1)
    numActor=numActorIn;
else
    numActor=2;
end

allLabels=["GNN cvKF" "GNN IMM" "JPDA cvKF" "JPDA IMM"];
f=figure();

differenceList=cell(4,1);
nameList=cell(4,1);
for method=1:4
    for type=1:4
        switch type
            case 1
                thisVar="Range";
            case 2
                thisVar="RangeAngles";
            case 3
                thisVar="Velocity";
            case 4
                thisVar="VelocityAngles";
        end
        thisDifference=allDifferences.(thisVar)(numActor,method,:);
        thisErrors=cell2mat(reshape(thisDifference,size(thisDifference,3),1,1));
        allErrors(type,method)=sqrt(mean(thisErrors.^2));
        statisticAnalysis(type,method)=datastats(thisErrors);
        for numTrack=1:size(thisDifference,3)
            differenceList{type,1}=[differenceList{type,1}; thisDifference{1,1,numTrack}];
            nameList{type,1}=[nameList{type,1}; repmat(allLabels(method),size(thisDifference{1,1,numTrack},1),1)];
        end
    end
end




subplot(2,1,1)
title(['Actor ' num2str(numActor) ': Range (difference)'])
hold on
grid on;
ylabel("[m]")
boxplot(differenceList{1,1},nameList{1,1});
% boxplot([allDifferences.Range{numActor,:,1}],'Labels',allLabels);

subplot(2,1,2)
title(['Actor ' num2str(numActor) ': Range direction (difference)'])
hold on
grid on;
ylabel("[rad]")
boxplot(differenceList{2,1},nameList{2,1});

h=figure();
subplot(2,1,1)
title(['Actor ' num2str(numActor) ': Velocity (difference)'])
hold on
grid on;
ylabel("[m/s]")
boxplot(differenceList{3,1},nameList{3,1});
% boxplot([allDifferences.Velocity{numActor,:,1}],'Labels',allLabels);

subplot(2,1,2)
title(['Actor ' num2str(numActor) ': Velocity direction (difference)'])
hold on
grid on;
ylabel("[rad]")
boxplot(differenceList{4,1},nameList{4,1});
% boxplot([allDifferences.Angles{numActor,:,1}],'Labels',allLabels);

rNames=["Range [m]" "Range direction [rad]" "Velocity[m/s]" "Velocity direction [rad]"];
RootMeanSquareError=table(allErrors(:,1),allErrors(:,2),allErrors(:,3),allErrors(:,4),'VariableNames',allLabels,'RowNames',rNames)

for i=1:4
   allMeans(:,i)=[statisticAnalysis(:,i).mean]';
   allStd(:,i)=[statisticAnalysis(:,i).std]';
end
Means=table(allMeans(:,1),allMeans(:,2),allMeans(:,3),allMeans(:,4),'VariableNames',allLabels,'RowNames',rNames)
StandardDeviations=table(allStd(:,1),allStd(:,2),allStd(:,3),allStd(:,4),'VariableNames',allLabels,'RowNames',rNames)



end


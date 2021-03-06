function detectionEvaluation(allDetections,numActors,allData)
thisDetsOrigins=[];
detsPerOrigin1=[];
for i=1:numel(allData)
    thisDetsOrigins=cellfun(@(x) x.ObjectAttributes{1,1}.TargetIndex,allData(i).ObjectDetections);
    for j=1:1+numActors
        if j==1
            detsPerOrigin1(i,j)=sum(thisDetsOrigins==-1);
        else
            detsPerOrigin1(i,j)=sum(thisDetsOrigins==j);
        end
    end
end
for k=1:1+numActors
    f(k)=figure();
    hold on;
    if(k==1)
        titleName="False alarms";
    else
        titleName=['Actor ' num2str(k)]; 
    end
    title(titleName);
    stairs(seconds([allData.Time]),detsPerOrigin1(:,k));
    numPossible(k)=sum(detsPerOrigin1(:,k)>0);
end


thisDetsOrigins=[];
detsPerOrigin2=[];
for i=1:numel(allDetections)
    thisDetsOrigins=cellfun(@(x) x.ObjectAttributes{1,1}.TargetIndex,allDetections{i,1});
    for j=1:1+numActors
        if j==1
            detsPerOrigin2(i,j)=sum(thisDetsOrigins==-1);
        else
            detsPerOrigin2(i,j)=sum(thisDetsOrigins==j);
        end
    end
end

for k=1:1+numActors
    figure(f(k).Number)
    hold on;
    ylabel(['Number of detections/' newline 'Number of detection clusters'])
    xlabel("Time")
    stairs(seconds([allData.Time]),detsPerOrigin2(:,k));
%     stairs(seconds([allData.Time]),detsPerOrigin1(:,k)-detsPerOrigin2(:,k));
    legend(["pre clustering" "post clustering"])%"difference"])
    f(k).Children(2).YLim(1)=0;
    if(k>1)
        evaluation=['Actor ' num2str(k) ': ' num2str(sum(detsPerOrigin2(:,k)>0)) ' of ' num2str(numPossible(k)) ' time steps detected ('...
            num2str(sum(detsPerOrigin2(:,k)>0)/numPossible(k)*100) ' percent)']
    end
end
end

methods=1:4;
close all
sceneFigure=figure();
set(gcf,'Visible','on')
thisAxes=axes('SortMethod','childorder');
isRunning=1;
scenario.SampleTime=sensorSampleTime;
restart(scenario);
thisTimeIndex=1;
thisX=[];
thisY=[];
while (isRunning)
    cla
    plot(scenario,'Parent',thisAxes);
    hold on;
    for i=methods
        for numTrack=(numActors+2):size(result{i,1},2)/4
            if(scenario.SimulationTime>4*sensorSampleTime)
                for j=0:5
                    thisX(j+1)=result{i,1}{seconds(scenario.SimulationTime-j*sensorSampleTime),(numTrack-1)*4+1};
                    thisY(j+1)=result{i,1}{seconds(scenario.SimulationTime-j*sensorSampleTime),(numTrack-1)*4+2};
                end
            else
                thisX=result{i,1}{seconds(scenario.SimulationTime),(numTrack-1)*4+1};
                thisY=result{i,1}{seconds(scenario.SimulationTime),(numTrack-1)*4+2};
            end
            
            
            switch i
                case 1
                    thisColor='k';
                case 2
                    thisColor='b';
                case 3
                    thisColor='y';
                case 4
                    thisColor='#D95319';
                    %                     thisColor='r';
            end
            %             scatter(thisX,thisY,[],thisColor)
            plot(thisAxes,thisX,thisY,'-.','Color',thisColor,'LineWidth',2.5,...
                'MarkerEdgeColor',thisColor, 'MarkerFaceColor',thisColor,'MarkerSize',5);
        end
    end
    drawnow;
    isRunning=advance(scenario);
end
restart(scenario);
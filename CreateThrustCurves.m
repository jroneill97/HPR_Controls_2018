function motorCluster = CreateThrustCurves(motorCluster,newTimeSpan)
% Redefines the motorCluster struct so that it fits in the 
% newly specified time span (newTimeSpan)
for i = 1:length(motorCluster)
    motorCluster(i).thrust = interp1(motorCluster(i).time,...
                                     motorCluster(i).thrust,newTimeSpan);
    for j = 1:length(newTimeSpan)
        if isnan(motorCluster(i).thrust(j))
          motorCluster(i).thrust(j) = 0;
        end
    end
    motorCluster(i).time = newTimeSpan;
end
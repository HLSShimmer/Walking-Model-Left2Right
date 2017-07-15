function [sensorMotion] = reduceMotionDataSize(footMotion, timeWindows)

    % iterate over fieldnames
    for fn = fieldnames(footMotion)'
       sensorMotion.(fn{1}) = footMotion.(fn{1})(timeWindows,:);
    end

end
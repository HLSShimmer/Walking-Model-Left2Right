function [qInTimeSeries,motionAccelInReferenceInTimeSeries,motionVelocityInReferenceInTimeSeries,motionPositionInReferenceInTimeSeries] = IntegrateInTimeSeries(gyro,accel,magnetic,qInitial,positionInitial,velocityInitial,gravityInReference,magneticInRefernce,para,methodSet,otherValues)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% integrate to get quaternion, accel, velocity, displacement %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% gyro                                    input      angular velocity, in sensor system, 3x1 vector
% accel                                   input      acceleration, in sensor system, 3x1 vector
% magnetic                                input      magnetic, in sensor system, 3x1 vector
% qInitial                                input      initial quaternion£¬[q0;q1;q2;q3]
% positionInitial                         input      initial position, global system (NED)
% accelIdeal                              input      ideal gravity in global system (NED)
% magneticIdeal                           input      ideal magnetic in global system (NED)
% para                                    input      some parameters be used
% methodSet                               input      method settings
% otherValues                             input      some additional values that might be used for specific method
% qInTimeSeries                           output     calculated quaternion in series
% motionAccelInReferenceInTimeSeries      output     calculated motion accel in global system
% motionVelocityInReferenceInTimeSeries   output     calculated motion velocity in global system
% motionPositionInReferenceInTimeSeries   output     calculated motion displacement in global system
%% declare some values
dataLength = size(gyro,1);             %length of data
qInTimeSeries = zeros(dataLength,4);
motionAccelInReferenceInTimeSeries = zeros(dataLength,3);
motionVelocityInReferenceInTimeSeries = zeros(dataLength,3);
motionPositionInReferenceInTimeSeries = zeros(dataLength,3);
%% each kind of motion
if para.motionCategories==1
    %% walk left to right, with ZUPT algorithm
    %%loop every step and get quaternion, motionAccel, motionVelocity, motionPosition
    insValues.kalmanStruct = otherValues.kalmanStruct;
    insValues.kalmanStruct.statePrevious = zeros(9,1);
    insValues.motionCategories = para.motionCategories;
    insValues.dt = para.dt;
    qInTimeSeries(1,:) = qInitial.';
    qTemp = qInitial;                         %represent the quaternion of current time 
    %%loop every step and integrate to get quaternion
    for i=2:dataLength
        %%calculate quaternion at current time
        if methodSet.quaternion == 1          % Madgwick
            qTemp = IntegrateQuat3(qTemp,gyro(i,:).',accel(i,:).',magnetic(i,:).',gravityInReference,magneticInRefernce,para);
        elseif methodSet.quaternion == 2      % Fourati
            qTemp = IntegrateQuat4(qTemp,gyro(i,:).',accel(i,:).',magnetic(i,:).',gravityInReference,magneticInRefernce,para);
        end
        %%INS function to get accel velocity and displacement
        insValues.zeroVelocityFlag = otherValues.zeroVelocityIndex(i);
        [motionAccel,motionVelocity,motionPosition,insValues] =  INSFunction(accel(i,:).',motionVelocityInReferenceInTimeSeries(i-1,:).',motionPositionInReferenceInTimeSeries(i-1,:).',qTemp,gravityInReference,insValues);
        %%storage
        qInTimeSeries(i,:) = qTemp.';
        motionAccelInReferenceInTimeSeries(i,:) = motionAccel.';
        motionVelocityInReferenceInTimeSeries(i,:) = motionVelocity.';
        motionPositionInReferenceInTimeSeries(i,:) = motionPosition.';
    end
end
% %% loop every step and integrate to get quaternion
% qInTimeSeries(1,:) = qInitial.';
% qTemp = qInitial;                   %represent the quaternion of current time 
% for i=2:dataNum
%     %%calculate quaternion at current time
%     if methodSet.quaternion == 1          % Madgwick
%         qTemp = IntegrateQuat3(qTemp,gyro(i,:).',accel(i,:).',magnetic(i,:).',gravityInReference,magneticInRefernce,para);
%     elseif methodSet.quaternion == 2      % Fourati
%         qTemp = IntegrateQuat4(qTemp,gyro(i,:).',accel(i,:).',magnetic(i,:).',gravityInReference,magneticInRefernce,para);
%     end
%     qInTimeSeries(i,:) = qTemp.';
% end
% for i=1:dataNum
%     %%transfer measured accel to global system
%     motionAccelInReferenceInTimeSeries(i,:) = CoordinateTransfer(accel(i,:).',qInTimeSeries(i,:).','b2r');
% end
% motionAccelInReferenceInTimeSeries = motionAccelInReferenceInTimeSeries - repmat(gravityInReference.',dataNum,1);
% %% double integration to get motion velocity & displacement
% if methodSet.accelIntegrate == 1         % Time Domain
%     [motionVelocityInReferenceInTimeSeries,motionPositionInReferenceInTimeSeries] = AccelIntegrate3(motionAccelInReferenceInTimeSeries,velocityInitial,positionInitial,para);
% elseif methodSet.accelIntegrate == 3     % ZUPT
%     [motionVelocityInReferenceInTimeSeries,motionPositionInReferenceInTimeSeries] = AccelIntegrate4(motionAccelInReferenceInTimeSeries,velocityInitial,positionInitial,para,otherValues.zeroVelocityIndex);
% end
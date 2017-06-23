function [motionAccel,motionVelocity,motionPosition,insValues] = INSFunction(accelMeasured,motionVelocityPrevious,motionPositionPrevious,quaternion,gravityInReference,insValues)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% INS function to get refined motionAceel motionVelocity & motionPosition in global frame
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%accelMeseaured                              input            meseaured accel by sensor
%motionVelocityPrevious                      input            global velcoity at previous sample
%motionPositionPrevious                      input            global position at previous sample
%quaternion                                  input            quaternion at current sample time
%gravityInReference                          input            gravity value in global frame
%insValues                                   input            some values need to use for INS
%motionAccel,motionVelocity,motionPosition   output           motion accel velocity displacement in global frame
%% declare some values

%% calculate motion Accel, velocity displacement according to the motion categories
if insValues.motionCategories==1             %%walk left to right, with ZUPT algorithm
%     motionAccel = CoordinateTransfer(accelMeasured,quaternion,'b2r') - gravityInReference;
%     motionVelocity = motionVelocityPrevious + motionAccel*insValues.dt;
%     motionPosition = motionPositionPrevious + motionVelocity*insValues.dt;
    if insValues.zeroVelocityFlag==1
        %normal integration
        accelRefined = accelMeasured - insValues.kalmanStruct.statePrevious(7:9);
        motionAccel = CoordinateTransfer(accelRefined,quaternion,'b2r') - gravityInReference;
        motionVelocity = motionVelocityPrevious + motionAccel*insValues.dt;
        motionPosition = motionPositionPrevious + motionVelocity*insValues.dt;
        %kalman process to refine 
        insValues.kalmanStruct.measurement = motionVelocity;
        insValues.kalmanStruct.transferMatrix = eye(9);
        insValues.kalmanStruct.transferMatrix(1:3,4:6) = insValues.dt*eye(3);
        insValues.kalmanStruct.transferMatrix(4:6,7:9) = insValues.dt*GetCoordinateTransferMatrix(quaternion,'b2r');
        insValues.kalmanStruct = KalmanFilterFunction(insValues.kalmanStruct,1);
        motionVelocity = motionVelocity - insValues.kalmanStruct.stateCurrent(4:6);
        motionPosition = motionPosition - insValues.kalmanStruct.stateCurrent(1:3);
        %kalman reset, because the errors have already been compensated
        insValues.kalmanStruct.stateCurrent(1:6) = 0;
        insValues.kalmanStruct.statePrevious = insValues.kalmanStruct.stateCurrent;
    else
        %kalman predict only, only propogate the covariance P
        insValues.kalmanStruct.transferMatrix = eye(9);
        insValues.kalmanStruct.transferMatrix(1:3,4:6) = insValues.dt*eye(3);
        insValues.kalmanStruct.transferMatrix(4:6,7:9) = insValues.dt*GetCoordinateTransferMatrix(quaternion,'b2r');
        insValues.kalmanStruct = KalmanFilterFunction(insValues.kalmanStruct,0);
        %normal integration
        motionAccel = CoordinateTransfer(accelMeasured,quaternion,'b2r') - gravityInReference;
        motionVelocity = motionVelocityPrevious + motionAccel*insValues.dt;
        motionPosition = motionPositionPrevious + motionVelocity*insValues.dt;
    end
end
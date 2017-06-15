%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% use quaternion to transfer acceleration to global frame,
%%% and HMM to detect zero-velocity, then make the integration
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;close all;clc;
%% flag of running HMM or load memory from previous time
FLAG_RUN_HMM = false;     %%true:running for new ; false:load memory
if FLAG_RUN_HMM
    %% load data
    load DataBase_WalkingFoot_shimmer6_10min_Disposed footMotion footStatic
    sensorMotion = footMotion;
    sensorStatic = footStatic;
    %% parameters for HMM
    ParaSetupWalkModel;
    %% get state sequence from HMM
    data = [sensorMotion.Accel_WideRange,sensorMotion.Gyro];
    [HMMstruct, stateEstimated, haltState] = WalkModelOptimization(data,para,methodSet);
    zeroVelocityIndex = find(stateEstimated==haltState);
    save WalkingMemoryStorage_shimmer6_WinKmeans13
else
    load WalkingMemoryStorage_shimmer6_WinKmeans13
end
%% draw HMM result
tSpan = 18000:19000;
figure(1)
subplot(211)
plot(tSpan,data(tSpan,para.selectedSignal),'r')
subplot(212)
plot(tSpan,stateEstimated(tSpan),'b')
%% parameter setup for sensor kinematics
ParaSetupSensorKinematics;
%% data processing, calculate quaternion, motion accel/velocity/displacement
positionInitial = [0;0;0];
velocityInitial = [0;0;0];
[quatSeries,motionAccelSeries,motionVelocitySeries,motionPositionSeries,dataStaticFiltered,dataMotionFiltered] = SensorKinematics(sensorStatic,sensorMotion,positionInitial,velocityInitial,para,methodSet,otherValues);
%% 
accel = zeros(size(motionAccelSeries));
for i=1:size(accel,1)
    accel(i,:) = CoordinateTransfer(motionAccelSeries(i,:).',quatSeries(i,:).','r2b');
end

index = ones(size(motionAccelSeries,1),1);
index(zeroVelocityIndex) = 0;
temp = motionAccelSeries(:,2);
temp = temp .* index;
sum(temp(temp>=0))
sum(temp(temp<0))

temp = accel(:,3);
temp = temp .* index;
sum(temp(temp>=0))
sum(temp(temp<0))
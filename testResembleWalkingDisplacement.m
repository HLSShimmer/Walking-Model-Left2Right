%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% use quaternion to transfer acceleration to global frame,
%%% and HMM to detect zero-velocity, then make the integration
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;close all;clc;
%test push github steph
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
    load WalkingMemoryStorage_shimmer5_WinKmeans13
end
%% draw HMM result
tSpan = 16000:20000;
figure(1)
subplot(211)
plot(tSpan,data(tSpan,para.selectedSignal),'r')
title('Gyro X')
subplot(212)
plot(tSpan,stateEstimated(tSpan),'b')
title('States of Steps')
%% parameter setup for sensor kinematics
ParaSetupSensorKinematics;
%% data processing, calculate quaternion, motion accel/velocity/displacement
positionInitial = [0;0;0];
velocityInitial = [0;0;0];
[quatSeries,motionAccelSeries,motionVelocitySeries,motionPositionSeries,dataStaticFiltered,dataMotionFiltered] = SensorKinematics(sensorStatic,sensorMotion,positionInitial,velocityInitial,para,methodSet,otherValues);
%print some information
belowZero = zeros(1,3);
aboveZero = zeros(1,3);
for i=1:3
    temp = motionAccelSeries(:,i);
    belowZero(i) = sum(temp(temp<=0));
    aboveZero(i) = sum(temp(temp>0));
end
belowZero
aboveZero
%% draw sensor kinematics result
figure(2)
plot(motionPositionSeries)
legend('Displacement X','Displacement Y','Displacement Z');
title('Displacement')

figure(3)
tSpan = 10000:19000;
subplot(311)
area(tSpan,motionAccelSeries(tSpan,1))
title('Accel X in Global Frame')
subplot(312)
plot(tSpan,quatSeries(tSpan,2))
title('Quaternion q1')
subplot(313)
temp = sqrt(sum(footMotion.Magnetic.^2,2));
plot(tSpan,temp(tSpan))
title('Magnetic Module')
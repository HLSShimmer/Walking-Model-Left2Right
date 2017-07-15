%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% use quaternion to transfer acceleration to global frame,
%%% and HMM to detect zero-velocity, then make the integration
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; close all; clc;

%% flag of running HMM or load memory from previous time
sensorName = 'shimmer5';
FLAG_RUN_HMM = true;     %%true:running for new ; false:load memory
tWindows = 10001:13000;
tSpan = 1:3000;

if FLAG_RUN_HMM
    %% load data
    %fileName = strcat('../Data/DataBase_WalkingFoot_',sensorName,'_10min_Disposed');
    fileName = '../Data/DataBase_WalkingFoot_Outdoor_shimmer5_5min_Disposed.mat';
    load(fileName,'footMotion', 'footStatic')
    sensorMotion = reduceMotionDataSize(footMotion, tWindows);
    %sensorMotion = footMotion;
    %sensorStatic = footStatic;
    
    %% parameters for HMM
    [para, methodSet] = ParaSetupWalkModel(sensorMotion.time);
    %% get state sequence from HMM
    data = [sensorMotion.Accel_WideRange,sensorMotion.Gyro];
    [HMMstruct, stateEstimated, stateNum, haltState] = WalkModelOptimization(data,para,methodSet);
    
    % arange the classification to fit the walking steps
    %[haltState, stateEstimated] = arangeWalking(data, HMMstruct, stateEstimated, stateNum, para.selectedSignal);
    % haltState
    
    figure(10)
    plot(tSpan, stateEstimated(tSpan), 'b')
    pause
    
    zeroVelocityIndex = find(stateEstimated==haltState);
    fileName = strcat('Outdoor_',sensorName,'_WinKmeans13');
    save(fileName);
else
    fileName = strcat('Outdoor_',sensorName,'_WinKmeans13');
    load(fileName)
end



%% draw HMM result
figure(1)
subplot(211)
plot(tSpan,data(tSpan,para.selectedSignal),'r')
title('Gyro X')
subplot(212)
plot(tSpan,stateEstimated(tSpan),'b')
title('States of Steps');
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

% figure(3)
% tSpan = 10000:19000;
% subplot(311)
% area(tSpan,motionAccelSeries(tSpan,1))
% title('Accel X in Global Frame')
% subplot(312)
% plot(tSpan,quatSeries(tSpan,2))
% title('Quaternion q1')
% subplot(313)
% temp = sqrt(sum(footMotion.Magnetic.^2,2));
% plot(tSpan,temp(tSpan))
% title('Magnetic Module')

figure(4)
subplot(311)
plot(motionVelocitySeries(:,1))
title('Velocity X');
subplot(312)
plot(motionVelocitySeries(:,2))
title('Velocity Y');
subplot(313)
plot(motionVelocitySeries(:,3))
title('Velocity Z');

figure(5)
% subplot(511)
% plot(tSpan,data(tSpan,para.selectedSignal),'r')
% title('Gyro X')
subplot(411)
plot(tSpan,motionAccelSeries(tSpan,3))
title('motion Accel')
subplot(412)
plot(tSpan,motionVelocitySeries(tSpan,3))
title('motion Velocity')
subplot(413)
plot(tSpan,motionPositionSeries(tSpan,3))
title('motion Displacement')
subplot(414)
plot(tSpan,stateEstimated(tSpan),'b')
title('States of Steps')
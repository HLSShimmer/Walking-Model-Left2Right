%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% use quaternion to transfer acceleration to global frame,
%%% and HMM to detect zero-velocity, then make the integration
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;close all;clc;
tSpan = 16000:20000;

%% flag of running HMM or load memory from previous time
sensorName = 'shimmer5';
FLAG_RUN_HMM = true;     %%true:running for new ; false:load memory
if FLAG_RUN_HMM
    %% load data
    fileName = strcat('DataBase_WalkingFoot_',sensorName,'_10min_Disposed');
    load(fileName,'footMotion', 'footStatic')
    sensorMotion = footMotion;
    sensorStatic = footStatic;
    %% parameters for HMM
    [para, methodSet] = ParaSetupWalkModel(sensorMotion.time);
    %% get state sequence from HMM
    data = [sensorMotion.Accel_WideRange,sensorMotion.Gyro];
    [HMMstruct, stateEstimated, stateNum] = WalkModelOptimization(data,para,methodSet);
    
    % arange the classification to fit the walking steps
    [haltState, stateEstimated] = arangeWalking(data, HMMstruct, stateEstimated, stateNum, para.selectedSignal);
    % haltState
    
    %figure(10)
    %plot(tSpan, stateSequenceKmeans(tSpan), 'r')
    %hold on
    %plot(tSpan, stateEstimated(tSpan), 'b')
    %hold off
    %pause
    
    zeroVelocityIndex = find(stateEstimated==haltState);
    fileName = strcat('WalkingMemoryStorage_',sensorName,'_WinKmeans13');
    save(fileName);
else
    fileName = strcat('WalkingMemoryStorage_',sensorName,'_WinKmeans13');
    load(fileName)
end
%% draw HMM result
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
tSpan = 10000:11500;
subplot(511)
plot(tSpan,data(tSpan,para.selectedSignal),'r')
title('Gyro X')
subplot(512)
plot(tSpan,motionAccelSeries(tSpan,1))
title('motion Accel')
subplot(513)
plot(tSpan,motionVelocitySeries(tSpan,1))
title('motion Velocity')
subplot(514)
plot(tSpan,motionPositionSeries(tSpan,1))
title('motion Displacement')
subplot(515)
plot(tSpan,stateEstimated(tSpan),'b')
title('States of Steps')
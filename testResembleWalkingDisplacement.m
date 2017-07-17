%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% use quaternion to transfer acceleration to global frame,
%%% and HMM to detect zero-velocity, then make the integration
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;close all;clc;
%% flag of running HMM or load memory from previous time
sensorName = 'shimmer6';
timeDuration = 4;
FLAG_RUN_HMM = false;     %%true:running for new ; false:load memory
if FLAG_RUN_HMM
    %% load data
    fileName = strcat('DataBase_WalkingFoot_Outdoor_20170706_',sensorName,'_',num2str(timeDuration),'min_Disposed');
    load(fileName,'footMotion', 'footStatic')
    sensorMotion = footMotion;
    sensorStatic = footStatic;
    %% parameters for HMM
    ParaSetupWalkModel;
    %% get state sequence from HMM
    data = [sensorMotion.Accel_WideRange,sensorMotion.Gyro];
    [HMMstruct, stateEstimated, haltState] = WalkModelOptimization(data,para,methodSet);
    zeroVelocityIndex = find(stateEstimated==haltState);
    fileName = strcat('WalkingMemoryStorage_20170706_',sensorName,'_Outdoor_',num2str(timeDuration),'min_WinKmeans13');
%     fileName = strcat('WalkingMemoryStorage_',sensorName,'_WinKmeans13');
    save(fileName);
else
    fileName = strcat('WalkingMemoryStorage_20170706_',sensorName,'_Outdoor_',num2str(timeDuration),'min_WinKmeans13');
%     fileName = strcat('WalkingMemoryStorage_',sensorName,'_WinKmeans13');
    load(fileName)
end
%% draw HMM result
tSpan = 19000:20000;
figure(1)
subplot(211)
plot(tSpan,data(tSpan,para.selectedSignal),'r')
title('Gyro X')
subplot(212)
plot(tSpan,stateEstimated(tSpan),'b')
title('States of Steps')
%% parameter setup for sensor kinematics
ParaSetupSensorKinematics;
zeroVelocityRange = [];
temp = [];
for i=1:length(otherValues.zeroVelocityIndex)
    if i == 1 && otherValues.zeroVelocityIndex(i) == 0
        continue;
    end
    if i == 1 && otherValues.zeroVelocityIndex(i) == 1
        temp = [temp,i];
        continue;
    end
    if otherValues.zeroVelocityIndex(i-1) == 0 && otherValues.zeroVelocityIndex(i) == 1
        temp = [temp,i];
        continue;
    end
    if otherValues.zeroVelocityIndex(i-1) == 1 && otherValues.zeroVelocityIndex(i) == 0
        temp = [temp,i];
        zeroVelocityRange = [zeroVelocityRange;temp];
        temp = [];
        continue;
    end
end
% interval = 8;
% for i=2:size(zeroVelocityRange,1)
%     otherValues.zeroVelocityIndex(zeroVelocityRange(i,1)-interval:zeroVelocityRange(i,1)) = 1;
% %     otherValues.zeroVelocityIndex(zeroVelocityRange(i,1)+1:zeroVelocityRange(i,1)+interval) = 0;
% %     otherValues.zeroVelocityIndex(zeroVelocityRange(i,2)-interval:zeroVelocityRange(i,2)-1) = 0;
% end
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
%% transfer measured magnetic to global frame
magnetiInReference = zeros(size(motionAccelSeries));
for i=1:size(magnetiInReference,1)
    temp = CoordinateTransfer(sensorMotion.Magnetic(i,:).',quatSeries(i,:).','b2r');
    magnetiInReference(i,:) = temp.';
end
%% find out the accel and velocity in each step
accelBelowZero = zeros(size(zeroVelocityRange,1),3);
accelAboveZero = zeros(size(accelBelowZero));
velocityEndPoint = accelAboveZero;
for i=1:size(zeroVelocityRange,1)
    for j=1:3
        temp = motionAccelSeries(zeroVelocityRange(i,1):zeroVelocityRange(i,2),j);
        accelBelowZero(i,j) = sum(temp(temp<=0));
        accelAboveZero(i,j) = sum(temp(temp>0));
        velocityEndPoint(i,j) = sum(temp);
    end
end
%% draw sensor kinematics result
figure(2)
tSpan = 1000:2500;
plot(motionPositionSeries(:,:))
legend('Displacement X','Displacement Y','Displacement Z');
title('Displacement')
% motionPositionSeries(end,:)
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
tSpan = 1000:2500;
subplot(511)
plot(tSpan,data(tSpan,para.selectedSignal),'r')
title('Gyro X')
subplot(512)
area(tSpan,motionAccelSeries(tSpan,1))
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

% figure(6)
% tSpan = 10000:15500;
% subplot(411)
% plot(tSpan,quatSeries(tSpan,1))
% title('Quat0')
% subplot(412)
% plot(tSpan,quatSeries(tSpan,2))
% title('Quat1')
% subplot(413)
% plot(tSpan,quatSeries(tSpan,3))
% title('Quat2')
% subplot(414)
% plot(tSpan,quatSeries(tSpan,4))
% title('Quat3')

figure(7)
tSpan = 1000:2500;
subplot(511)
area(tSpan,motionAccelSeries(tSpan,1))
title('motion Accel X')
subplot(512)
plot(tSpan,motionVelocitySeries(tSpan,1),'r')
title('motion Velocity X')
subplot(513)
area(tSpan,motionAccelSeries(tSpan,2))
title('motion Accel Y')
subplot(514)
plot(tSpan,motionVelocitySeries(tSpan,2),'b')
title('motion Velocity Y')
subplot(515)
plot(tSpan,stateEstimated(tSpan),'b')
title('States of Steps')

figure(8)
tSpan = 11604:12683;
subplot(311)
area(tSpan,motionAccelSeries(tSpan,3))
title('motion Accel Z')
subplot(312)
plot(tSpan,motionVelocitySeries(tSpan,3),'r')
title('motion Velocity Z')
subplot(313)
plot(tSpan,motionPositionSeries(tSpan,3))
title('motion Position Z')

figure(9)
plot(motionPositionSeries(:,1),motionPositionSeries(:,2))
axis equal

figure(10)
plot3(magnetiInReference(:,1),magnetiInReference(:,2),magnetiInReference(:,3),'*')
axis equal
grid on

figure(11)
subplot(311)
plot(magnetiInReference(:,1))
ylabel('magnetic X')
title('Magnetic')
subplot(312)
plot(magnetiInReference(:,2))
ylabel('magnetic Y')
subplot(313)
plot(magnetiInReference(:,3))
ylabel('magnetic Z')
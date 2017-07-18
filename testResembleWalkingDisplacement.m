%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% use quaternion to transfer acceleration to global frame,
%%% and HMM to detect zero-velocity, then make the integration
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all; close all; clc;

%% flag of running HMM or load memory from previous time
sensorName = 'shimmer6_4min';
FLAG_RUN_HMM = false;     %%true:running for new ; false:load memory
tWindows = 10001:15000;
tSpan = 10000:11000;
timeDuration = 4;

%fileName = ['../Data/DataBase_WalkingFoot_', sensorName, '_10min_Disposed'];
%fileName = ['../Data/DataBase_WalkingFoot_Outdoor_', sensorName, '_5min_Disposed.mat'];
fileName    = ['../Data/DataBase_WalkingFoot_Outdoor_20170706_', sensorName, '_Disposed'];
figName     = ['../Data/FIG/DataBase_WalkingFoot_Outdoor_20170706_', sensorName, '_Disposed'];
fileNameHMM = ['../Data/HMM/Outdoor_', sensorName, '_WinKmeans13'];

if FLAG_RUN_HMM,
    %% load data
    load(fileName,'footMotion', 'footStatic')
    sensorStatic = footStatic;
%     sensorMotion = reduceMotionDataSize(footMotion, tWindows);
    sensorMotion = footMotion;

    %% parameters for HMM
    [para, methodSet] = ParaSetupWalkModel(sensorMotion.time) ;
    %% get state sequence from HMM
    data = [sensorMotion.Accel_WideRange,sensorMotion.Gyro];
    
    [HMMstruct, stateEstimated, haltState] = WalkModelOptimization(data,para,methodSet);
    zeroVelocityIndex = find(stateEstimated==haltState);
    
    save(fileNameHMM);
else
    load(fileNameHMM);
end

%% draw HMM result
figure(1)
subplot(211)
plot(tSpan,data(tSpan,para.selectedSignal))
title('Gyro X')
subplot(212)
plot(tSpan,stateEstimated(tSpan),'b')
title('States of Steps');
print('-dpng','-r300',[figName, '_fig1.png'])


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
plot(motionPositionSeries(:,:))
legend('Displacement X','Displacement Y','Displacement Z');
title('Displacement')
print('-dpng','-r300',[figName, '_fig2.png'])

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
figure(3)
plot(motionPositionSeries(:,1),motionPositionSeries(:,2))
axis equal
xlabel('East')
ylabel('North')
print('-dpng','-r300',[figName, '_fig3.png'])

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
print('-dpng','-r300',[figName, '_fig4.png'])

figure(5)
subplot(411)
area(tSpan,motionAccelSeries(tSpan,2))
title('motion Accel')
subplot(412)
plot(tSpan,motionVelocitySeries(tSpan,2))
title('motion Velocity')
subplot(413)
plot(tSpan,motionPositionSeries(tSpan,2))
title('motion Displacement')
subplot(414)
plot(tSpan,stateEstimated(tSpan),'b')
title('States of Steps')
print('-dpng','-r300',[figName, '_fig5.png'])

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
subplot(311)
area(tSpan,motionAccelSeries(tSpan,3))
title('motion Accel Z')
subplot(312)
plot(tSpan,motionVelocitySeries(tSpan,3),'r')
title('motion Velocity Z')
subplot(313)
plot(tSpan,motionPositionSeries(tSpan,3))
title('motion Position Z')
print('-dpng','-r300',[figName, '_fig8.png'])

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
print('-dpng','-r300',[figName, '_fig11.png'])

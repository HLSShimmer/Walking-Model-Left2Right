%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% test global frame, to find it is stable or not
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;close all;clc;
%% load memory from data memory
sensorName = 'shimmer2';
timeDuration = 4;
fileName = strcat('WalkingMemoryStorage_',sensorName,'_Outdoor_',num2str(timeDuration),'min_WinKmeans13');
load(fileName)
%% parameter setup for sensor kinematics
ParaSetupSensorKinematics;
%% data processing, calculate quaternion, motion accel/velocity/displacement
positionInitial = [0;0;0];
velocityInitial = [0;0;0];
[quatSeries,motionAccelSeries,motionVelocitySeries,motionPositionSeries,dataStaticFiltered,dataMotionFiltered] = SensorKinematics(sensorStatic,sensorMotion,positionInitial,velocityInitial,para,methodSet,otherValues);
%% transfer measured magnetic to global frame
magneticStaticInReference = zeros(size(sensorStatic.Magnetic));
magneticMotionInReference = zeros(size(motionAccelSeries));
for i=1:size(magneticStaticInReference,1)
    temp = CoordinateTransfer(sensorStatic.Magnetic(i,:).',quatSeries(1,:).','b2r');
    magneticStaticInReference(i,:) = temp.';
end
for i=1:size(magneticMotionInReference,1)
    temp = CoordinateTransfer(sensorMotion.Magnetic(i,:).',quatSeries(i,:).','b2r');
    magneticMotionInReference(i,:) = temp.';
end
%% get euller angles from quaternion
[yaw,pitch,roll] = GetEullerAngles(quatSeries);
[yawShimmer,pitchShimmer,rollShimmer] = GetEullerAngles(sensorMotion.Quat9DOF_WideRange);
yaw = yaw/pi*180;
pitch = pitch/pi*180;
roll = roll/pi*180;
yawShimmer = yawShimmer/pi*180;
pitchShimmer = pitchShimmer/pi*180;
rollShimmer = rollShimmer/pi*180;
%% draw result
tSpan = 9000:12000;
figure(1)
subplot(311)
plot(yaw(tSpan))
hold on
plot(yawShimmer(tSpan))
title('Yaw')
legend('Calculated','Shimmer')
subplot(312)
plot(pitch(tSpan))
hold on
plot(pitchShimmer(tSpan))
title('Pitch')
legend('Calculated','Shimmer')
subplot(313)
plot(roll(tSpan))
hold on
plot(rollShimmer(tSpan))
title('Roll')
legend('Calculated','Shimmer')

figure(3)
subplot(311)
plot(magneticMotionInReference(:,1))
ylabel('magnetic X')
title('Magnetic')
subplot(312)
plot(magneticMotionInReference(:,2))
ylabel('magnetic Y')
subplot(313)
plot(magneticMotionInReference(:,3))
ylabel('magnetic Z')

figure(4)
plot(motionPositionSeries(tSpan,1),motionPositionSeries(tSpan,2))
axis equal

figure(5)
subplot(411)
area(tSpan,motionAccelSeries(tSpan,1))
title('motion Accel X')
subplot(412)
area(tSpan,motionVelocitySeries(tSpan,1))
title('motion Velocity X')
subplot(413)
plot(tSpan,motionPositionSeries(tSpan,1))
title('motion Displacement X')
subplot(414)
plot(tSpan,stateEstimated(tSpan),'b')
title('States of Steps')

figure(6)
subplot(411)
area(tSpan,motionAccelSeries(tSpan,2))
title('motion Accel Y')
subplot(412)
area(tSpan,motionVelocitySeries(tSpan,2))
title('motion Velocity Y')
subplot(413)
plot(tSpan,motionPositionSeries(tSpan,2))
title('motion Displacement Y')
subplot(414)
plot(tSpan,stateEstimated(tSpan),'b')
title('States of Steps')

figure(7)
plot(motionPositionSeries(:,1),motionPositionSeries(:,2))
axis equal

figure(8)
subplot(411)
plot(quatSeries(tSpan,1))
ylabel('q0')
title('Quaternion')
subplot(412)
plot(quatSeries(tSpan,2))
ylabel('q1')
subplot(413)
plot(quatSeries(tSpan,3))
ylabel('q2')
subplot(414)
plot(quatSeries(tSpan,4))
ylabel('q3')

figure(9)
temp1 = magneticMotionInReference;
temp2 = temp1(tSpan,:);
temp1(tSpan,:) = [];
plot3(magneticStaticInReference(:,1),magneticStaticInReference(:,2),magneticStaticInReference(:,3),'k+','MarkerSize',6)
hold on
plot3(temp1(:,1),temp1(:,2),temp1(:,3),'b*','MarkerSize',2)
hold on
plot3(temp2(:,1),temp2(:,2),temp2(:,3),'r*','MarkerSize',8)
legend('static','motion others','motion selected')
title('Magnetic Displayed in 3D')
axis equal
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')
axis([0 1.2 -0.3 0.3 -1.3 0])

figure(10)
temp1(:,1) = sqrt(sum(temp1(:,1:2).^2,2));
temp1(:,2) = 0;
temp2(:,1) = sqrt(sum(temp2(:,1:2).^2,2));
temp2(:,2) = 0;
plot(temp1(:,1),temp1(:,3),'b*','MarkerSize',2)
hold on
plot(temp2(:,1),temp2(:,3),'r*','MarkerSize',8)
hold on
plot(magneticStaticInReference(:,1),magneticStaticInReference(:,3),'k+','MarkerSize',6)
legend('motion others','motion selected','static')
title('Magnetic in X-Z plane')

figure(11)
plot(sqrt(sum(magneticMotionInReference.^2,2)))
title('Magnetic Module')
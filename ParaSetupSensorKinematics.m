%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% set up the parameters that needed by SensorKinematics function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% if there exist para and methodSet, delete first
clear para methodSet
%% parameters
para.dt = 1/100.51;             %sample step
% TD Filter
para.h1 = 7*para.dt;
para.r = 2000;
% Quaternion Madgwick
para.alpha = 300;                %augmentation to acount for noise from accelerometer and magnetometer
para.beta = 0.35;               %divergence rate of quaternion
% Quaternion Fourati
para.lamda = 2;                 %convergence factor
para.k = 0.3;                   %filter factor
% Frequency Domain Integration
para.fResolution = 0.0003;
% Frequency Filter
para.fmin_velocity = 0.14;                    %cutoff frequency, lower & upper bound
% para.fmin_velocity = fmin_velocity(count);
para.fmax_velocity = 50;
para.fTarget_velocity = 0.09;                 %target frequncy
para.integrateAccuracy_velocity = 0.95;       %integrate accuracy
para.fmin_position = 0.07;                 %cutoff frequency, lower & upper bound 0.1236
% para.fmin_position = fmin_position(count);
para.fmax_position = 100;
para.fTarget_position = 0.09;             %target frequncy
para.integrateAccuracy_position = 0.95;   %integrate accuracy
para.minPeakDistance = 0.002;
para.minPeakHeight = 10;
% Time Domain Integration
para.lamdaV = 0.001;             %threshold of variance
para.lamdaM = 20;               %threshold of interval
para.populationSize = 10;
%% mothed setting
methodSet.dataFilter = 1;         %1-TD
methodSet.quaternion = 1;         %1-Madgwick,2-Fourati
methodSet.accelIntegrate = 3;     %1-TimeDomain;3-zero velocity
%% other values
otherValues.zeroVelocityIndex = zeroVelocityIndex;
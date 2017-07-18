% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % set up the parameters that needed by SensorKinematics function
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % if there exist para and methodSet, delete first
% clear para methodSet
% % parameters
para.dt = 1 / 100.51;             % sample step
% para.dt = 1 / 50.21;
% TD Filter
para.h1 = 7 * para.dt;
para.r = 2000;
% average filter
para.windowSize = 11;
% Quaternion Madgwick
para.alpha = 50;               % augmentation to acount for noise from accelerometer and magnetometer
para.beta = 0.3;               % divergence rate of quaternion
% Quaternion Fourati
para.lamda = 2;                 % convergence factor
para.k = 0.3;                   % filter factor
% Frequency Domain Integration
para.fResolution = 0.0003;
% Frequency Filter
para.fmin_velocity = 0.14;                    % cutoff frequency, lower & upper bound
% para.fmin_velocity = fmin_velocity(count);
para.fmax_velocity = 50;
para.fTarget_velocity = 0.09;                 % target frequncy
para.integrateAccuracy_velocity = 0.95;       % integrate accuracy
para.fmin_position = 0.07;                 % cutoff frequency, lower & upper bound 0.1236
% para.fmin_position = fmin_position(count);
para.fmax_position = 100;
para.fTarget_position = 0.09;             % target frequncy
para.integrateAccuracy_position = 0.95;   % integrate accuracy
para.minPeakDistance = 0.002;
para.minPeakHeight = 10;
% Time Domain Integration
para.lamdaV = 0.001;             % threshold of variance
para.lamdaM = 20;               % threshold of interval
para.populationSize = 10;
% % mothed setting
methodSet.dataFilter = 2;          % 1 - TD£¬2 - Average
methodSet.quaternion = 1;          % 1 - Madgwick, 2 - Fourati
methodSet.accelIntegrate = 3;      % 1 - TimeDomain; 3 - ZUPT
methodSet.integration = 1;         % 1 - euller; 2 - trapz; 3 - simpson; 4 - RK4;
% % motion categories
para.motionCategories = 1;        % 1 - Walk Left2Right;
% % other values
temp = zeros(size(data, 1), 1);
temp(zeroVelocityIndex) = 1;
otherValues.zeroVelocityIndex = temp;
% kalman struct settings
% fileName = strcat('../Data/', sensorName, '_NoiseMeasurement');
% fileName = strcat('../Data/shimmer5_NoiseMeasurement');
% load(fileName)
kalmanStruct.statePrevious = zeros(9, 1);  % ¦Är, ¦Äv, ¦Äa
kalmanStruct.statePredict = zeros(9, 1);
kalmanStruct.stateCurrent = zeros(9, 1);
kalmanStruct.transferMatrix = eye(9);     % state transfer matrix
kalmanStruct.observeMatrix = zeros(3, 9);  % observe matrix, only velocity can be observed during zero velocity state
kalmanStruct.observeMatrix(1, 4) = 1;
kalmanStruct.observeMatrix(2, 5) = 1;
kalmanStruct.observeMatrix(3, 6) = 1;
kalmanStruct.measurement = zeros(3, 1);    % error measurement,  velocity during zero - velocity state
kalmanStruct.P_previous = zeros(9, 9);         % a posterior error covariance matrix
kalmanStruct.P_predict = zeros(9, 9);
kalmanStruct.P_current = zeros(9, 9);
kalmanStruct.covarianceQ = zeros(9, 9);
kalmanStruct.covarianceR = zeros(3, 3);
% kalmanStruct.covarianceQ(7: 9, 7: 9) = cov(noise_Accel_WideRange);
% kalmanStruct.covarianceQ(4: 6, 4: 6) = sqrt(cov(noise_Accel_WideRange)) * para.dt;
% kalmanStruct.covarianceQ(7, 7) = var(noise_Accel_WideRange(:, 1));
% kalmanStruct.covarianceQ(8, 8) = var(noise_Accel_WideRange(:, 2));
% kalmanStruct.covarianceQ(9, 9) = var(noise_Accel_WideRange(:, 3));
kalmanStruct.covarianceQ(7, 7) = 0.01;
kalmanStruct.covarianceQ(8, 8) = 0.01;
kalmanStruct.covarianceQ(9, 9) = 0.01;
% kalmanStruct.covarianceQ(4, 4) = (sqrt(kalmanStruct.covarianceQ(7, 7)) * para.dt) ^ 2;
% kalmanStruct.covarianceQ(5, 5) = (sqrt(kalmanStruct.covarianceQ(8, 8)) * para.dt) ^ 2;
% kalmanStruct.covarianceQ(6, 6) = (sqrt(kalmanStruct.covarianceQ(9, 9)) * para.dt) ^ 2;
kalmanStruct.covarianceR(1, 1) = 0.01;
kalmanStruct.covarianceR(2, 2) = 0.01;
kalmanStruct.covarianceR(3, 3) = 0.01;
otherValues.kalmanStruct = kalmanStruct;

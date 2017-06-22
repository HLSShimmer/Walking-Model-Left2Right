function kalmanStruct = KalmanFilterFunction(kalmanStruct,flag)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Kalman Filter function (Predict & Update)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%flag                  input            indicate doing predict&update or predict only, 0-predict, 1-predict&update
%kalmanStruct          input&output     kalman struct, includes state, transfer matrix, observe matrix, covariance...
%% declare some values
stateNum = size(kalmanStruct.stateCurrent,1);
%% doing kalman function based on flag
if nargin==1
    %% kalman filter predict
    kalmanStruct.statePredict  = kalmanStruct.transferMatrix*kalmanStruct.statePrevious;
    kalmanStruct.P_predict = kalmanStruct.transferMatrix*kalmanStruct.P_previous*kalmanStruct.transferMatrix.' + kalmanStruct.covarianceQ;
    %% kalman filter update
    K = kalmanStruct.P_predict*kalmanStruct.observeMatrix.' / (kalmanStruct.covarianceR + kalmanStruct.observeMatrix*kalmanStruct.P_predict*kalmanStruct.observeMatrix.');
    kalmanStruct.stateCurrent = kalmanStruct.statePredict + K*(kalmanStruct.measurement - kalmanStruct.observeMatrix*kalmanStruct.statePredict);
    kalmanStruct.P_current = (eye(stateNum) - K*kalmanStruct.observeMatrix)*kalmanStruct.P_predict;
    kalmanStruct.P_previous = kalmanStruct.P_current;
    return;
end
if flag==1
    %% kalman filter predict
    kalmanStruct.statePredict  = kalmanStruct.transferMatrix*kalmanStruct.statePrevious;
    kalmanStruct.P_predict = kalmanStruct.transferMatrix*kalmanStruct.P_previous*kalmanStruct.transferMatrix.' + kalmanStruct.covarianceQ;
    %% kalman filter update
    K = kalmanStruct.P_predict*kalmanStruct.observeMatrix.' / (kalmanStruct.covarianceR + kalmanStruct.observeMatrix*kalmanStruct.P_predict*kalmanStruct.observeMatrix.');
    kalmanStruct.stateCurrent = kalmanStruct.statePredict + K*(kalmanStruct.measurement - kalmanStruct.observeMatrix*kalmanStruct.statePredict);
    kalmanStruct.P_current = (eye(stateNum) - K*kalmanStruct.observeMatrix)*kalmanStruct.P_predict;
    kalmanStruct.P_previous = kalmanStruct.P_current;
elseif flag==0
    kalmanStruct.P_predict = kalmanStruct.transferMatrix*kalmanStruct.P_previous*kalmanStruct.transferMatrix.' + kalmanStruct.covarianceQ;
    kalmanStruct.P_previous = kalmanStruct.P_predict;
end
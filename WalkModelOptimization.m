function [HMMstructOptimized, stateEstimated, haltState] = WalkModelOptimization(data,para,methodSet)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% left-to-right walking model optimization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%data                   input        walking data needed
%para                   input        parameters for methods
%methodSet              input        method settings
%HMMstructOptimized     output       HMM struct
%stateEstimated         output       estimated state for each sample time
%haltState              output       the state which means zero velocity
%% declare some values
%HMM struct paras
stateNum = 4;                %state number
observeDimension = 1;        %dimension of observation
optPara = para.optPara;
%% filter
dataFiltered = FilterData(data,para.dt,methodSet.dataFilter,para);
<<<<<<< HEAD
% size(dataFiltered)
% tSpan = 16000:16200;
% figure
% plot(tSpan,dataFiltered(tSpan,1),'r')
% hold on;
% plot(tSpan,data(tSpan,1),'b')
% hold off;
% title('dataFiltered')
% pause
=======
>>>>>>> cea80e1bb23a9a62cf254868edb20341ecbab128

%% generate HMM struct
%%% comment : the kmeans is applied on all the 6 vectorial data (accel and gyro)
%%% maybe not a good strategy: kmeans shoul may be applied on the same
%%% signal as HMM
tic;
[HMMstruct,~,haltState] = InitializeWalkingModel(dataFiltered,stateNum,para);
HMMstruct,haltState
toc, 
%% generate HMM model
model = HMM();
%% optimal para settings
model = model.SetModel(HMMstruct);
model = model.SetOptPara(optPara);
%% optimize model
observeSequence = dataFiltered(:,para.selectedSignal);
tic
[model,HMMstructOptimized,stateEstimated,flag] = model.ModelOptimization(observeSequence);
toc
%% display the result of optimization
if flag==0
    disp('Optimization finished and has reached the maximum iteration')
elseif flag==1
    disp('Optimization finished and meets the tolerance')
end
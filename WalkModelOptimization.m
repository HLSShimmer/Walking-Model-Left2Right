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
observeDimension = 1;        %dimension of observance
optPara = para.optPara;
%% filter
dataFiltered = FilterData(data,para.dt,methodSet.dataFilter,para);
%% generate HMM struct
[HMMstruct,~,haltState] = InitializeWalkingModel(dataFiltered,stateNum,para);
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
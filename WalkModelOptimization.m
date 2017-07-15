function [HMMstructOptimized, stateEstimated, stateNum, haltState] = WalkModelOptimization(data,para,methodSet)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% left-to-right walking model optimization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%data                   input        walking data needed
%para                   input        parameters for methods
%methodSet              input        method settings
%HMMstructOptimized     output       HMM struct
%stateEstimated         output       estimated state for each sample time
%% declare some values
%HMM struct paras
stateNum = 4;                %state number
observeDimension = 1;        %dimension of observation
optPara = para.optPara;
%% filter
%dataFiltered = FilterData(data,para.dt,methodSet.dataFilter,para);
dataFiltered = data;

% size(dataFiltered)
% tSpan = 16000:16200;
% figure
% plot(tSpan,dataFiltered(tSpan,1),'r')
% hold on;
% plot(tSpan,data(tSpan,1),'b')
% hold off;
% title('dataFiltered')
% pause

%% generate HMM struct
%%% comment : the kmeans is applied on all the 6 vectorial data (accel and gyro)
%%% maybe not a good strategy: maybe kmeans shoul be applied on the same
%%% signal as HMM?
[HMMstruct,~, haltState] = InitializeWalkingModel(dataFiltered,stateNum,para);

%% generate HMM model
observeLength = length(dataFiltered);
model = HMM(observeLength, HMMstruct.N);
%% optimal para settings
model = model.SetModel(HMMstruct);
model = model.SetOptPara(optPara);

%% optimize model
observeSequence = dataFiltered(:,para.selectedSignal);

% Data for Stephane's programs
f = fopen('/Users/MacBook_Derrode/Documents/temp/testHaoyu/data.txt', 'w');
if (f~=-1)   
    fprintf(f, '1 %d\n', length(observeSequence));
    for i=1:length(observeSequence),
        fprintf(f, '%f\n', observeSequence(i));
    end
    fclose(f);
end

[model,HMMstructOptimized,stateEstimated,flag] = model.ModelOptimization(observeSequence);
HMMstructOptimized.A

% Data for Stephane's programs
f = fopen('/Users/MacBook_Derrode/Documents/temp/testHaoyu/kmeans.txt', 'w');
if (f~=-1)   
    fprintf(f, '1 %d\n', length(stateEstimated));
    for i=1:length(stateEstimated),
        fprintf(f, '%f\n', stateEstimated(i));
    end
    fclose(f);
end

%% display the result of optimization
if flag==0
    disp('Optimization finished and has reached the maximum iteration')
elseif flag==1
    disp('Optimization finished and meets the tolerance')
end
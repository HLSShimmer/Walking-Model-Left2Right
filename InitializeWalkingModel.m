function [HMMstruct,stateSequence, haltState] = InitializeWalkingModel(data,stateNum,para)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% get an initial HMM model by KMeans with the input data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%data                input           raw data of walking
%stateNum            input           the number of stat
%para                input           structure with some constant value
%HMMstruct           output          structure with initialized HMM model
%stateSequence       output          re-arranged state sequence from KMeans method

%% declare some values
signalNum = size(data,2);      %% the number of different signals
dataLength = size(data,1);     %% length of data, number of samples

% initial classification by Kmeans
% comment : I choose to replicate the kmeans, to avoid possible abnormal
% convergence (the kmeans with the best convergence value is selected)
opts = statset('Display','final');
stateSequenceKmeans = kmeans(data,stateNum,'Replicates', 15, 'start', 'uniform', 'Options', opts);

% Estimation of a priori vectors and matrices from the kmeans classif
jointProbabilityKmeans      = zeros(stateNum, stateNum);
transitProbabilityKmeans    = zeros(stateNum, stateNum);
stationaryProbabilityKmeans = zeros(       1, stateNum);
TABLE = tabulate(stateSequenceKmeans);
stationaryProbabilityKmeans = TABLE(: , 3) ./ 100;
for i=2:dataLength,
    jointProbabilityKmeans(stateSequenceKmeans(i-1), stateSequenceKmeans(i)) = jointProbabilityKmeans(stateSequenceKmeans(i-1), stateSequenceKmeans(i)) + 1;
end

jointProbabilityKmeans   = jointProbabilityKmeans./(dataLength-1);
transitProbabilityKmeans = jointProbabilityKmeans./repmat(stationaryProbabilityKmeans,1,stateNum);
%stationaryProbabilityKmeans, jointProbabilityKmeans, transitProbabilityKmeans, 
%%sum(stationaryProbabilityKmeans), sum(sum(jointProbabilityKmeans)), sum(sum(transitProbabilityKmeans))
%pause

%% searching for the state order
stateSequence = zeros(size(stateSequenceKmeans));
jointProbability      = zeros(stateNum, stateNum);
transitProbability    = zeros(stateNum, stateNum);
stationaryProbability = zeros(       1, stateNum);
stateSeries = 1:stateNum;
stateTransferOrder = zeros(stateNum,1);
[~,stateTransferOrder(1)]=min(stationaryProbabilityKmeans);     %start with the minimum possibility state, it's better
stateSeries(stateSeries==stateTransferOrder(1)) = [];           %when find a correct state order, delete the state in stateSeries
for i = 1:stateNum-1
    temp = transitProbabilityKmeans(stateTransferOrder(i),stateSeries);
    [~,index] = max(temp);
    stateTransferOrder(i+1) = stateSeries(index);
    stateSeries(stateSeries==stateTransferOrder(i+1)) = [];     %when find a correct state order, delete the state in stateSeries
end
%stateTransferOrder
%% re-arrange the state according to the order, also change the joint/transit/stationary probability
for i=1:stateNum
    stateSequence(stateSequenceKmeans==stateTransferOrder(i)) = i;
    stationaryProbability(i) = stationaryProbabilityKmeans(stateTransferOrder(i));
end
for i=1:stateNum
    for j=1:stateNum
        jointProbability(i,j) = jointProbabilityKmeans(stateTransferOrder(i),stateTransferOrder(j));
        transitProbability(i,j) = transitProbabilityKmeans(stateTransferOrder(i),stateTransferOrder(j));
    end
end
%% calculate halt state
averageABS =zeros(stateNum,1);
for i=1:stateNum
    temp = data(stateSequence==i,:);
    for j=1:signalNum
        averageABS(i) = averageABS(i) + abs(mean(temp(:,j)));
    end
end
[~,index] = min(averageABS);
haltState = index;
%stationaryProbability, jointProbability, transitProbability, 
%pause,

% Correction of the a priori matrices to be a Left-Right model
[stationaryProbability, jointProbability, transitProbability] = correctLeftRight(stationaryProbability, jointProbability, transitProbability);
stationaryProbability, jointProbability, transitProbability, 
%pause
%sum(stationaryProbabilityKmeans), sum(sum(jointProbabilityKmeans)), sum(sum(transitProbabilityKmeans))
%pause

%% generate HMM struct
selectedSignal = para.selectedSignal;               %the index of selected signal
HMMstruct.N = stateNum;
HMMstruct.M = 1;
HMMstruct.observePDFType = 'CONTINUOUS_GAUSSIAN';   %% CONTINUOUS_GAUSSIAN, DISCRET
HMMstruct.transitMatrixType = 'LEFT2RIGHT';         %% NORMAL, LEFT2RIGHT
HMMstruct.A = transitProbability;
HMMstruct.initialStateProbability = stationaryProbability;
HMMstruct.B.mixtureNum = 1;
HMMstruct.B.weights    = ones(stateNum,1);
HMMstruct.B.mu         = cell(stateNum,1);
HMMstruct.B.sigma      = cell(stateNum,1);
HMMstruct.B.PDF        = cell(stateNum,1);
for i=1:stateNum
    HMMstruct.B.mu{i}    = mean(data(stateSequence==i,selectedSignal));
    HMMstruct.B.sigma{i} = zeros(length(selectedSignal),length(selectedSignal),1);
    HMMstruct.B.sigma{i}(:,:,1) = cov(data(stateSequence==i,selectedSignal));
    HMMstruct.B.PDF{i}   = gmdistribution(HMMstruct.B.mu{i},HMMstruct.B.sigma{i},HMMstruct.B.weights(i,:));
end

HMMstruct.A, 
HMMstruct.initialStateProbability,
HMMstruct.B.mu, 

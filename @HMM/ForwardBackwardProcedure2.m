function obj = ForwardBackwardProcedure2(obj)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% forward and backward procedure
%%%% calculate the needed ??and ?? in Baum-Welch Algorithm
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% obj           input&output    object
% gamma         output          probability of state at n given by the entire observe sequence
% ksai          output          probability of state at n and n+1 given by the entire observe sequence
%% declare some variables
N = obj.HMMstruct.N;
initialStateProbability = obj.HMMstruct.initialStateProbability;
observeLength = length(obj.observeSequence);
observeProbability = zeros(observeLength,N);
states = 1:N; 
%% forward
observeProbability(1,:) = obj.GetObserveProbability(states,1);
obj.alpha(1,:) = observeProbability(1,:).*initialStateProbability./sum(observeProbability(1,:).*initialStateProbability);

for i = 2:observeLength
    forwardNextState = obj.alpha(i-1,:)*obj.HMMstruct.A;
    observeProbability(i,:) = obj.GetObserveProbability(states,i);
    temp = observeProbability(i,:).*forwardNextState;  
    obj.Sn(i-1) = sum(temp);
    obj.alpha(i,:) = temp/obj.Sn(i-1);
end
%% backward
obj.beta(observeLength,:) = 1;
for i=observeLength-1:-1:1
    observeProbabilityMatrix = repmat(observeProbability(i+1,:),N,1);
    backwardResultMatrix = repmat(obj.beta(i+1,:),N,1);
    obj.beta(i,:) = sum(obj.HMMstruct.A.*observeProbabilityMatrix.*backwardResultMatrix,2).'/obj.Sn(i);
end

% gamma
for i=1:observeLength
    obj.gamma(i,:) = obj.alpha(i,:).*obj.beta(i,:);
end

% ksai
for i=1:observeLength-1
    forwardResultMatrix      = repmat(obj.alpha(i,:).',1,N);
    backwardResultMatrix     = repmat(obj.beta(i+1,:),N,1);
    observeProbabilityMatrix = repmat(observeProbability(i+1,:),N,1);
    obj.ksai(:,:,i)          = (forwardResultMatrix.*obj.HMMstruct.A.*backwardResultMatrix.*observeProbabilityMatrix)./obj.Sn(i);
end
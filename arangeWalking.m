function [haltState, rearangedStateSequence] = arangeWalking(data, HMMstruct, stateEstimated, stateNum, selectedSignal)

    signalNum = size(data,2);
    rearangedStateSequence = zeros(size(stateEstimated));
    
    % %% searching for the state order
    stateSeries = 1:stateNum;
    stateTransferOrder = zeros(stateNum,1);
    [~,stateTransferOrder(1)]=min(HMMstruct.initialStateProbability);     %start with the minimum possibility state, it's better

    stateSeries(stateSeries==stateTransferOrder(1)) = [];           %when find a correct state order, delete the state in stateSeries
    for i = 1:stateNum-1
        temp = HMMstruct.A(stateTransferOrder(i),stateSeries);
         [~,index] = max(temp);
         stateTransferOrder(i+1) = stateSeries(index);
         stateSeries(stateSeries==stateTransferOrder(i+1)) = [];     %when find a correct state order, delete the state in stateSeries
     end
    %% re-arrange the state according to the order, also change the joint/transit/stationary probability
    transitProbability    = zeros(stateNum, stateNum);
    stationaryProbability = zeros(       1, stateNum);
    for i=1:stateNum
        rearangedStateSequence(stateEstimated==stateTransferOrder(i)) = i;
        stationaryProbability(i) = HMMstruct.initialStateProbability(stateTransferOrder(i));
    end
    for i=1:stateNum
        for j=1:stateNum
            transitProbability(i,j) = HMMstruct.A(stateTransferOrder(i),stateTransferOrder(j));
        end
    end
    %stationaryProbability, transitProbability, 
    %stateTransferOrder, pause

    %% calculate halt state
    averageABS = zeros(stateNum,1);
    for i=1:stateNum
        temp = data(rearangedStateSequence==i,:);
        for j=1:signalNum
            averageABS(i) = averageABS(i) + abs(mean(temp(:,j)));
        end
    end
    [~,haltState] = min(averageABS);

    
    %% update HMM struct
    HMMstruct.A = transitProbability;
    for i=1:stateNum
        HMMstruct.B.mu{i}    = mean(data(rearangedStateSequence==i,selectedSignal));
        HMMstruct.B.sigma{i} = zeros(1,1,1);
        HMMstruct.B.sigma{i}(1,1,1) = var(data(rearangedStateSequence==i,selectedSignal));
        HMMstruct.B.PDF{i}   = gmdistribution(HMMstruct.B.mu{i},HMMstruct.B.sigma{i},HMMstruct.B.weights(i,:));
    end
    HMMstruct.initialStateProbability = stationaryProbability';

end


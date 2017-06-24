function [haltState] = arangeWalking(HMMstruct, stateEstimated, stateNum)

% %% searching for the state order
stateSeries = 1:stateNum;
stateTransferOrder = zeros(stateNum,1);
[~,stateTransferOrder(1)]=min(HMMstruct.initialStateProbability);     %start with the minimum possibility state, it's better

pause 
% stateSeries(stateSeries==stateTransferOrder(1)) = [];           %when find a correct state order, delete the state in stateSeries
% for i = 1:stateNum-1
%     temp = transitProbabilityKmeans(stateTransferOrder(i),stateSeries);
%     [~,index] = max(temp);
%     stateTransferOrder(i+1) = stateSeries(index);
%     stateSeries(stateSeries==stateTransferOrder(i+1)) = [];     %when find a correct state order, delete the state in stateSeries
% end
% %% re-arrange the state according to the order, also change the joint/transit/stationary probability
% for i=1:stateNum
%     stateSequence(stateSequenceKmeans==stateTransferOrder(i)) = i;
%     stationaryProbability(i) = stationaryProbabilityKmeans(stateTransferOrder(i));
% end
% for i=1:stateNum
%     for j=1:stateNum
%         jointProbability(i,j) = jointProbabilityKmeans(stateTransferOrder(i),stateTransferOrder(j));
%         transitProbability(i,j) = transitProbabilityKmeans(stateTransferOrder(i),stateTransferOrder(j));
%     end
% end
% %stationaryProbability, jointProbability, 
% transitProbability, 
% stateTransferOrder, pause

% figure(10)
% %plot(tSpan, stateSequenceKmeans(tSpan), 'r')
% hold on
% plot(tSpan, stateSequence(tSpan), 'b')
% hold off
% pause
%stateSequence(1:50), stateSequenceKmeans(1:50)
%sum(stationaryProbability), sum(sum(jointProbability)), sum(sum(transitProbability))
%pause

%% calculate halt state
% averageABS =zeros(stateNum,1);
% for i=1:stateNum
%     temp = data(stateSequence==i,:);
%     for j=1:signalNum
%         averageABS(i) = averageABS(i) + abs(mean(temp(:,j)));
%     end
% end
% [~,index] = min(averageABS);
% haltState = index;

end


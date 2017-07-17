function [stationaryP_LR, jointP_LR, transitP_LR] = correctLeftRight(stationaryP_Orig, jointP_Orig, transitP_Orig)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Correct the matrices to make the model Left-Right
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%stationaryP_Orig   input           Original init law 
%jointP_Orig        input           Original joint matrix 
%transitP_Orig      input           Original transit matrix 
%stationaryP_LR     input           Left-Right init law 
%jointP_LR          input           Left-Right joint matrix 
%transitP_LRs       input           Left-Right transit matrix 


stationaryP_LR = zeros(size(stationaryP_Orig));
jointP_LR      = zeros(size(jointP_Orig));
transitP_LR    = zeros(size(transitP_Orig));

stateNumber = size(stationaryP_LR, 2);

stationaryP_LR = stationaryP_Orig;
for k=1:stateNumber,
    [val, indice] = max(transitP_Orig(k,:));
    indicepun = mod(indice,stateNumber)+1;
 
    transitP_LR(k, indice)    = transitP_Orig(k, indice);
    transitP_LR(k, indicepun) = 1.-transitP_LR(k, indice);
    
    jointP_LR(k, :) = transitP_LR(k, :) .* stationaryP_LR(k);
end






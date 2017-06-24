%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% set up the parameters that needed by WalkModel
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [para, methodSet] = ParaSetupWalkModel(timeStampVect)
    %% parameters
    para.dt = mean(timeStampVect(2:end)-timeStampVect(1:end-1));               %sample step
    para.selectedSignal = 4;          %the index of signal that be used to identify zero-velocity
    para.windowSize = 7;              %window for filter
    para.optPara.maxIter = 2;
    para.optPara.ErrorTolerance = 2e-4;
    para.optPara.ChangingTolerance = 2e-4;
    %% method setting
    methodSet.dataFilter = 2;         %1-TD, 2-mean
end
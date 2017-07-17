%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% set up the parameters that needed by WalkModel
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% if there exist para and methodSet, delete first
clear para methodSet
%% parameters
para.dt = 1/100.51;               %sample step
para.selectedSignal = 4;          %the index of signal that be used for identify zero-velocity
para.windowSize = 7;              %window for filter
para.optPara.maxIter = 60;
para.optPara.ErrorTolerance = 0.01;
para.optPara.ChangingTolerance = 1.7e-5;
%% mothed setting
methodSet.dataFilter = 2;         %1-TD, 2-mean
function [A,B,initialStateProbability] = CalculateUpdateInformation(obj)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Calculate Update of A, B, ?? %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%obj                       input       object
%A                         output      updated transition matrix
%B                         output      updated discret observe distribute or GMM struct
%initialStateProbability   output      updated initialState probability
%% declare some variables
N = obj.HMMstruct.N;
M = obj.HMMstruct.M;
observeLength = length(obj.observeSequence);
initialStateProbability = zeros(1,N);
A = zeros(N,N);
B = zeros(N,M);
%% Update A
if strcmp(obj.HMMstruct.transitMatrixType,'NORMAL')
    for i=1:N
        for j=1:N
            A(i,j) = sum(obj.ksai(i,j,:))/sum(obj.gamma(1:observeLength-1,i));
        end
    end
elseif strcmp(obj.HMMstruct.transitMatrixType,'LEFT2RIGHT')
    DELTA = zeros(N,1);
    for i=1:N
        ipun = mod(i,N)+1;
        DELTA(i) = obj.ksai(i,ipun,:)/(sum(obj.ksai(i,i,:))+obj.ksai(i,ipun,:));
        A(i,i)    = 1 - DELTA(i);
        A(i,ipun) = DELTA(i);
    end
end
%% Update B
if strcmp(obj.HMMstruct.observePDFType,'DISCRET')
    for i=1:N
        for j=1:M
            B(i,j) = sum(obj.gamma(observeSequence==j,i))/sum(obj.gamma(:,i));
        end
    end
elseif strcmp(obj.HMMstruct.observePDFType,'CONTINUOUS_GAUSSIAN')
    B = obj.UpdateBContinuous();
end
%% Update ??
for i=1:N
    initialStateProbability(i) = obj.gamma(1,i)/sum(obj.gamma(1,:));
end
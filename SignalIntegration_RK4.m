function result = SignalIntegration_RK4(signal,dt,initial,Type)
%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% RK4 integration %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%
% signal               input      signal to be integrate,  rows represent sample time step, cols represent how many type of signals need to be integrated
% dt                   input      sample time step
% initial              input      initial value��ʼֵ
% Type                 input      'V' to calculate velocity or 'P' for postion 
% result               output     result of integration, same size of signal
%% declare some values
signalLength = length(signal);
signalNum = size(signal,2);
result = zeros(signalLength,signalNum);
%% RK4
if Type=='P'
    for i=1:signalNum
        result(1,i) = initial(i);
        result(2,i) = result(1,i) + dt*(signal(1,i)+4*signal(2,i)+signal(3,i))/6;
        result(3,i) = result(2,i) + dt*(signal(2,i)+4*signal(3,i)+signal(4,i))/6;
        for j=4:signalLength
        K1 = signal(j-3,i);
        K2 = signal(j-2,i);
        K3 = signal(j-1,i);
        K4 = signal(j,i);
        result(j,i) = result(j-1,i) + dt*(K1+2*K2+2*K3+K4)/6;
        end
    end
elseif Type=='V'
    for i=1:signalNum
        result(1,i) = initial(i);
        result(2,i) = result(1,i) + dt*(signal(1,i)+4*signal(2,i)+signal(3,i))/6;
        result(3,i) = result(2,i) + dt*(signal(2,i)+4*signal(3,i)+signal(4,i))/6;
        for j=4:signalLength
            if signal(j,i)~=0
                K1 = signal(j-3,i);
                K2 = signal(j-2,i);
                K3 = signal(j-1,i);
                K4 = signal(j,i);
                result(j,i) = result(j-1,i) + dt*(K1+2*K2+2*K3+K4)/6;
            else
                result(j,i) = result(j-1,i);
            end
        end
    end
end
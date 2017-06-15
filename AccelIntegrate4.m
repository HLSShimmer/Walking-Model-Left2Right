function [motionVelocity,motionPosition] = AccelIntegrate4(motionAccel,initialVelocity,initialPosition,para,zeroVelocityIndex)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% accel double integration, 
%%% use zero velocity to reset velocity when doing the integration
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% motionAccel                     input      motion accel,each row represent sample at each time step, each col represent xyz axis
% intialVelocity                  input      initial value of velocity
% initialPosition                 input      initial value of position
% para                            input      some parameters be used
% zeroVelocityIndex               input      index that indicate
% motionVelocity, motionPosition  output     result of double integration
%% declare some values
dataLength = size(motionAccel,1);
signalNum = size(motionAccel,2);
dt = para.dt;
motionVelocity = zeros(dataLength,signalNum);
motionPosition = zeros(dataLength,signalNum);
zeroVelocityMask = zeros(dataLength,1);
zeroVelocityMask(zeroVelocityIndex) = 1;        %%where 1 means the velocity is zero, should reset the velocity
%% by using the mask, doing the double integration, RK4 method
%%accel to velocity
for i=1:signalNum
    motionVelocity(1,i) = initialVelocity(i);   %%initial value of velocity
    if zeroVelocityMask(1)==1                   %%if is zero-velocity state at first time, reset velocity 
        motionVelocity(1,i) = 0;
    end
    motionVelocity(2,i) = motionVelocity(1,i) + dt*(motionAccel(1,i)+4*motionAccel(2,i)+motionAccel(3,i))/6;
    if zeroVelocityMask(2)==1                   %%if is zero-velocity state at second time, reset velocity
        motionVelocity(2,i) = 0;
    end
    motionVelocity(3,i) = motionVelocity(2,i) + dt*(motionAccel(2,i)+4*motionAccel(3,i)+motionAccel(4,i))/6;
    if zeroVelocityMask(3)==1                   %%if is zero-velocity state at third time, reset velocity
        motionVelocity(3,i) = 0;
    end
    for j=4:dataLength
        if zeroVelocityMask(j)==1
            motionVelocity(j,i) = 0;            %%if is zero-velocity state at j-th time, reset velocity
        else
            K1 = motionAccel(j-3,i);            %%RK4
            K2 = motionAccel(j-2,i);
            K3 = motionAccel(j-1,i);
            K4 = motionAccel(j,i);
            motionVelocity(j,i) = motionVelocity(j-1,i) + dt*(K1+2*K2+2*K3+K4)/6;
        end
    end
end
%%velocity to position
for i=1:signalNum
    motionPosition(1,i) = initialPosition(i);   %%initial value of position
    motionPosition(2,i) = motionPosition(1,i) + dt*(motionVelocity(1,i)+4*motionVelocity(2,i)+motionVelocity(3,i))/6;
    motionPosition(3,i) = motionPosition(2,i) + dt*(motionVelocity(2,i)+4*motionVelocity(3,i)+motionVelocity(4,i))/6;
    for j=4:dataLength
        K1 = motionVelocity(j-3,i);             %%RK4
        K2 = motionVelocity(j-2,i);
        K3 = motionVelocity(j-1,i);
        K4 = motionVelocity(j,i);
        motionPosition(j,i) = motionPosition(j-1,i) + dt*(K1+2*K2+2*K3+K4)/6;
    end
end
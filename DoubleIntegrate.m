function [motionVelocity, motionPosition] = DoubleIntegrate(motionAccel,motionAccelPrevious,motionVelocityPrevious,motionPositionPrevious,flag,dt)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% one step integrate for different method
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% motionAccel               input         accel at current time
% motionAccelPrevious       input         motion Accel in Previous time
% motionVelocityPrevious    input         motion Velocity in Previous time
% motionPositionPrevious    input         motion Position in Previous time
% flag                      input         1-euller;2-trapz;3-simpson;4-RK4;
% dt                        input         sample time
% motionVelocity            output        motion velocity at current time
% motionPosition            output        motion position at current time
%% declare some values
previousNum = size(motionAccelPrevious,2);
%% integrate
if previousNum==1
    if flag==1
        motionVelocity = motionVelocityPrevious + motionAccel*dt;
        motionPosition = motionPositionPrevious + motionVelocity*dt;
    else
        motionVelocity = motionVelocityPrevious + (motionAccel + motionAccelPrevious)*dt/2;
        motionPosition = motionPositionPrevious + (motionVelocity + motionVelocityPrevious)*dt/2;
    end
else
    if flag==1
        motionVelocity = motionVelocityPrevious(:,end) + motionAccel*dt;
        motionPosition = motionPositionPrevious(:,end) + motionVelocity*dt;
    elseif flag==2
        motionVelocity = motionVelocityPrevious(:,end) + (motionAccel + motionAccelPrevious)*dt/2;
        motionPosition = motionPositionPrevious(:,end) + (motionVelocity + motionVelocityPrevious)*dt/2;
    elseif flag==3
        motionVelocity = motionVelocityPrevious(:,end) + (motionAccelPrevious(:,1) + 4*motionAccelPrevious(:,2) + motionAccel)*dt/6;
        motionPosition = motionPositionPrevious(:,end) + (motionVelocityPrevious(:,1) + 4*motionVelocityPrevious(:,2) + motionVelocity)*dt/6;
    elseif flag==4
        motionVelocity = motionVelocityPrevious(:,end) + (motionAccelPrevious(:,1) + 2*motionAccelPrevious(:,2) + 2*motionAccelPrevious(:,3) + motionAccel)*dt/6;
        motionPosition = motionPositionPrevious(:,end) + (motionVelocityPrevious(:,1) + 2*motionVelocityPrevious(:,2) + 2*motionVelocityPrevious(:,3) + motionVelocity)*dt/6;
    end
end
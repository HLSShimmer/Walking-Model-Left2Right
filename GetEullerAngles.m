function [yaw,pitch,roll] = GetEullerAngles(quaternion)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Get euller angles from quaternion
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%quaternion          input           Nx4,N is the sample number, 4 indicates [q0,q1,q2,q3]
%yaw,pitch,roll      output          euller angles

%% declare some values
N = size(quaternion,1);
yaw = zeros(N,1);
pitch = zeros(N,1);
roll = zeros(N,1);
%% calculate euller angles
for i=1:N
    q0 = quaternion(i,1);
    q1 = quaternion(i,2);
    q2 = quaternion(i,3);
    q3 = quaternion(i,4);
    yaw(i) = atan2(2*(q0*q3+q1*q2),1-2*(q2^2+q3^2));
    pitch(i) = asin(2*(q0*q2-q1*q3));
    roll(i) = atan2(2*(q0*q1+q2*q3),1-2*(q1^2+q2^2));
end
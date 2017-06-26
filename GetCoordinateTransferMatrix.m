function transferMatrix = GetCoordinateTransferMatrix(quaternion,direction)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% get coordinate transfer matrix by the given quaternion and direction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%quaternion           input          quaternion representing the rotation from sensor system to global system, 4x1 vector
%direction            input          rotation direction, r2b: global to sensor£»b2r: sensor to global
%transferMatrix       output         coordinate transfer matrix
%% generate transfer matrix
q0 = quaternion(1);
q1 = quaternion(2);
q2 = quaternion(3);
q3 = quaternion(4);
transferMatrix = zeros(3,3);
if strcmp(direction,'b2r')
    transferMatrix(1,1) = q0^2 + q1^2 - q2^2 - q3^2;
    transferMatrix(1,2) = 2*q1*q2 - 2*q0*q3;
    transferMatrix(1,3) = 2*q0*q2 + 2*q1*q3;
    
    transferMatrix(2,1) = 2*q0*q3 + 2*q1*q2;
    transferMatrix(2,2) = q0^2 - q1^2 + q2^2 - q3^2;
    transferMatrix(2,3) = -2*q0*q1 + 2*q2*q3;
    
    transferMatrix(3,1) = -2*q0*q2 + 2*q1*q3;
    transferMatrix(3,2) = 2*q0*q1 + 2*q2*q3;
    transferMatrix(3,3) = q0^2 - q1^2 - q2^2 + q3^2;
elseif strcmp(direction,'r2b')
    transferMatrix(1,1) = q0^2 + q1^2 - q2^2 - q3^2;
    transferMatrix(1,2) = 2*q1*q2 + 2*q0*q3;
    transferMatrix(1,3) = -2*q0*q2 + 2*q1*q3;
    
    transferMatrix(2,1) = -2*q0*q3 + 2*q1*q2;
    transferMatrix(2,2) = q0^2 - q1^2 + q2^2 - q3^2;
    transferMatrix(2,3) = 2*q0*q1 + 2*q2*q3;
    
    transferMatrix(3,1) = 2*q0*q2 + 2*q1*q3;
    transferMatrix(3,2) = -2*q0*q1 + 2*q2*q3;
    transferMatrix(3,3) = q0^2 - q1^2 - q2^2 + q3^2;
end
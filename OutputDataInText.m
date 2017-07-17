clear all;clc;close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Ouput data in txt file format
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% declare some valubles
dataBaseName = 'WalkingMemoryStorage_shimmer1_WinKmeans13';
fileName = 'DataBase_WalkingFoot_shimmer6_10min.txt';
titleName = {'Accel X','Accel Y','Accel Z','Gyro X','Gyro Y','Gyro Z','Step States'};
databaseSeries = 1:6;
%% output data txt
for i=databaseSeries(1):databaseSeries(end)
    %file name
    dataBaseName(29) = num2str(databaseSeries(i));
    %load data base
    command = strcat('load',32,dataBaseName,32,'footMotion',32,'stateEstimated');
    eval(command);
    data = [footMotion.Accel_WideRange.';footMotion.Gyro.';stateEstimated.'];
    %output to txt
    fileName(29) = num2str(databaseSeries(i));
    fileHandle = fopen(fileName,'w');
    for j=1:length(titleName)-1
        string = strcat('# ',titleName{j},'\r\n');
        fprintf(fileHandle,string);
        fprintf(fileHandle,'%.5f\t',data(j,:));
        string = strcat('#################\r\n\r\n');
        fprintf(fileHandle,string);
    end
    string = strcat('# ',titleName{end},'\r\n');
    fprintf(fileHandle,string);
    fprintf(fileHandle,'%d\t',data(end,:));
    string = strcat('#################\r\n\r\n');
    fprintf(fileHandle,string);
    fclose(fileHandle);
end
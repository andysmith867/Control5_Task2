%%  fuzzyTask1Controller

% Syntax: 

% Inputs:
%
% Outputs:          


% author: Andrew Smith
% date: 21/02/2022
% last modification time: 21/03/2022, 1500
function [checkPointCounter,voltages] = fuzzyTaskOneController(booleanAtCheckPoint, checkPointCounter, error, sensorOut)
if booleanAtCheckPoint == true
    text = ('Checkpoint reached! \n');
    fprintf(text);
    checkPointCounter = checkPointCounter +1 ;
    voltages = [0 0];
elseif booleanAtCheckPoint == false
    fuzzyController = readfis('taskTwoController.fis');
    inputs = [ error sensorOut(:,2) sensorOut(:,1)];
    voltages = evalfis(fuzzyController,inputs) ;
end


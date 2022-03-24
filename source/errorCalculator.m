%%  errorCALCULATOR 

% Syntax: [error] = errorCalculator(newHeadingAngle, currentHeadingAngle)

% Inputs:
% newHeadingAngle: heading angle required           radians
% currentHeadingAngle: current path of the robot    radians
% Outputs:          
% error: difference between the required and current path 
%        of the robot                               radians

% author: Andrew Smith
% date: 21/02/2022
% last modification time: 21/03/2022, 1850

function [error] = errorCalculator(newHeadingAngle, currentHeadingAngle)
error = newHeadingAngle - currentHeadingAngle;
if error < - pi
    error = error + 2*pi;
elseif error > pi
    error = error - 2*pi;
end

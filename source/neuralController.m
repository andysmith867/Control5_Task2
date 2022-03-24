% inputs = weights, thresholds, sensors and input voltages
% outputs = motor voltages
% input data tye = array
% output data type = struct
% date last modified = 11/03/2022 1325
% author = andy smith

% weights = [ w1 w2 w3 w4];
% thresholds = [mr ml];

function [voltage_out] = neuralController(weights, thresholds, sensorOut, voltage_in,k)
sensor_l = sensorOut(1,1);
sensor_r = sensorOut(1,2);
w1 = weights(1,1);
w2 = weights(1,2);
w3 = weights(1,3);
w4 = weights(1,4);

threshold_mr = thresholds(1,1);
threshold_ml = thresholds(1,2);

if sensor_l*w3+sensor_r*w1 > threshold_mr && sensor_r*w2 + sensor_l*w4 > threshold_ml
    voltage_out.left = 6;
    voltage_out.right =6;
elseif sensor_l*w3+sensor_r*w1 <= threshold_mr && sensor_r*w2 + sensor_l*w4 > threshold_ml
    voltage_out.left = voltage_in.left *1;
    voltage_out.right =voltage_in.right*0;
    %if sensor_l >= 1
        voltage_out.left = voltage_in.left *6;
        voltage_out.right =voltage_in.right*6;
   % end
elseif sensor_l*w3+sensor_r*w1 > threshold_mr && sensor_r*w2 + sensor_l*w4 <= threshold_ml
    voltage_out.left = voltage_in.left *0;
    voltage_out.right =voltage_in.right*1;
  %  if sensor_r >= 1
        %voltage_out.left = voltage_in.left *6;
        %voltage_out.right =voltage_in.right*6;
    %end
elseif sensor_l*w3+sensor_r*w1 <= threshold_mr && sensor_r*w2 + sensor_l*w4 <= threshold_ml
    voltage_out.left = voltage_in.left *1;
    voltage_out.right =voltage_in.right*-1;
end
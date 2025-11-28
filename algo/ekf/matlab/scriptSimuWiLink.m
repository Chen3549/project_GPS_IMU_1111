% load saved data

dataPathGNSS = '.\test\walk_test\2025_11_27_10_01_sunny\data\gps_AT6558R_2025_11_27_10_01_56.csv';
dataPathIMU = '.\test\walk_test\2025_11_27_10_01_sunny\data\imu_2025_11_27_10_01_56.csv';
dataGNSS = extractDataGNSS(dataPathGNSS);
dataIMU = extractDataIMU(dataPathIMU);

[dataGNSSInterpolated, dataIMUInterpolated] = linearInterpolatedDataGNSS(dataGNSS, dataIMU);

drawGNSSRawVSInterpolatedTrack(dataGNSS(:, 2)', dataGNSS(:, 3)', dataGNSSInterpolated(:, 2)', dataGNSSInterpolated(:, 3)', '.\test\walk_test\2025_11_27_10_01_sunny\picture\');

% parse data position(x/y/z)
s_x =  dataGNSSInterpolated(2, :);
s_y =  dataGNSSInterpolated(3, :);
s_z =  dataGNSSInterpolated(4, :);

% parse data speed(x/y/z)
v_x =  dataGNSSInterpolated(5, :);
v_y =  dataGNSSInterpolated(6, :);
v_z =  dataGNSSInterpolated(7, :);

% delta T
dt = 0.02;

% run simulation according to dataGPS dataIMU
%  Kalman filter tracking position and speed
% X
s_x_flitered = simpleKalman(s_x, v_x);
% Y
s_x_flitered = simpleKalman(s_x, v_x);
% Z
s_x_flitered = simpleKalman(s_x, v_x);

% 
% estimation of a (via dataIMU)
% 



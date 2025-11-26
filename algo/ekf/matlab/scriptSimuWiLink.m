% load saved data

dataPathGNSS = 'D:\home\user\chenqiang\ekf\matlab\demo_gnss.csv';
dataPathIMU = 'D:\home\user\chenqiang\ekf\matlab\demo_imu.csv';
dataGNSS = extractDataGNSS(dataPathGNSS);
dataIMU = extractDataIMU(dataPathIMU);

disp(dataGNSS);
disp(dataIMU);

[dataGNSSInterpolated, dataIMUInterpolated] = linearInterpolatedDataGNSS(dataGNSS, dataIMU);

disp(dataGNSSInterpolated);
disp(dataIMUInterpolated);

% % parse data position(x/y/z)
% s_x =  [];
% s_y =  [];
% s_z =  [];
% 
% % parse data speed(x/y/z)
% v_x =  [];
% v_y =  [];
% v_z =  [];
% 
% % delta T
% dt = 0.02;
% 
% % run simulation according to dataGPS dataIMU
% %  Kalman filter tracking position and speed
% % X
% s_x_flitered = simpleKalman(s_x, v_x);
% % Y
% s_x_flitered = simpleKalman(s_x, v_x);
% % Z
% s_x_flitered = simpleKalman(s_x, v_x);
% 
% % 
% % estimation of a (via dataIMU)
% % 



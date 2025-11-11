% % EKF导航INS测试脚本
% 
% % 创建EKF对象
% ekf = ekfNavINS();
% 
% % 模拟初始化数据
% time = 0;
% vn = 0; % NORTH
% ve = 0; % EAST[
% vd = 0; % DOWN
% lat = 45.0 * pi/180; % Latitude
% lon = -93.0 * pi/180; % Lontitude ?
% alt = 300; % Altitude
% 
% p = 0.01; 
% q = 0.02;
% r = 0.03;
% 
% % accelarator ?
% ax = 0.1;
% ay = -0.05;
% az = 9.8;
% 
% % magnetic ?
% hx = 0.2;
% hy = 0.1;
% hz = -0.3;
% 
% % 初始化EKF
% ekf.ekf_update(time, vn, ve, vd, lat, lon, alt, p, q, r, ax, ay, az, hx, hy, hz);
% 
% % 模拟更新循环
% for i = 1:100
%     time = time + 1e6; % 1秒后 time unit: us
% 
%     % 模拟传感器数据
%     % velocity
%     vn = 100 + 0.1*randn;
%     ve = 2 + 0.1*randn;
%     vd = 0.5 + 0.1*randn;
%     % position
%     lat = lat + 1e-6;
%     lon = lon + 1e-6;
%     alt = alt + 0.1;
%     % raw pitch yaw
%     p = 0.01 + 0.001*randn;
%     q = 0.02 + 0.001*randn;
%     r = 0.03 + 0.001*randn;
%     % acc
%     ax = 0.1 + 0.01*randn;
%     ay = -0.05 + 0.01*randn;
%     az = 9.8 + 0.01*randn;
%     % magnetic
%     hx = 0.2 + 0.01*randn;
%     hy = 0.1 + 0.01*randn;
%     hz = -0.3 + 0.01*randn;
% 
%     % 更新EKF
%     ekf.ekf_update(time, vn, ve, vd, lat, lon, alt, p, q, r, ax, ay, az, hx, hy, hz);
% 
%     % 获取状态
%     fprintf('Iteration %d:\n', i);
%     fprintf('  Position: lat=%.6f, lon=%.6f, alt=%.2f\n', ...
%             ekf.getLatitude_rad()*180/pi, ekf.getLongitude_rad()*180/pi, ekf.getAltitude_m());
%     fprintf('  Attitude: roll=%.2f, pitch=%.2f, yaw=%.2f\n', ...
%             ekf.getRoll_rad()*180/pi, ekf.getPitch_rad()*180/pi, ekf.getHeading_rad()*180/pi);
%     fprintf('  Velocity: vn=%.2f, ve=%.2f, vd=%.2f\n', ...
%             ekf.getVelNorth_ms(), ekf.getVelEast_ms(), ekf.getVelDown_ms());
% end

draw_ekf_data('./test/gnss.csv', true)
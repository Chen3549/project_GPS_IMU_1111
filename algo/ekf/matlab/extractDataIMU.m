function dataIMU = extractDataIMU(dataIMUPath)
%EXTRACTDATAIMU
% input: data IMU path
%
% output:
% data matrix IMU:
% dataIMU format: time(UTC, μs)
%                 acceleration-x(m/s^2) / acceleration-y(m/s^2) / acceleration-z(m/s^2) /
%                 angular speed-x(rad/s) / angular speed-y(rad/s) / angular speed-z(rad/s) /
%                 magnetic-x(μT) / magnetic-y(μT) / magnetic-z(μT)
%

% read raw data csv
% time ax(g) ay(g) az(g) wx(deg/s) wy(deg/s) wz(deg/s) hx(μT) hy(μT) hz(μT)
rawDataIMU = readtable(dataIMUPath);
% g to m/s^2
rawDataIMU.ax_mps2 = rawDataIMU.ax * 9.8;
rawDataIMU.ay_mps2 = rawDataIMU.ay * 9.8;
rawDataIMU.az_mps2 = rawDataIMU.az * 9.8;
% deg/s to rad/s
rawDataIMU.wx_rps = deg2rad(rawDataIMU.wx);
rawDataIMU.wy_rps = deg2rad(rawDataIMU.wx);
rawDataIMU.wz_rps = deg2rad(rawDataIMU.wx);

dataIMU = table2array(rawDataIMU(:, {'time', 'ax_mps2', 'ay_mps2', 'az_mps2', 'wx_rps', 'wy_rps', 'wz_rps', 'hx', 'hy', 'hz'}));
end
function drawIMUAccelerationData(accX, accY, accZ, savePath)
% draw IMU acceleration data, the pictures will be saved as
%   'savePath/acc-x.svg' & 'savePath/acc-y.svg'
%   & 'savePath/acc-z.svg' if 'savePath' passed
%
% input: acceleration-X (m/s^2)
%        acceleration-Y (m/s^2)
%        acceleration-Z (m/s^2)
%        path where the pictures will be saved
% 
% Example:
%   drawIMUAccelerationData(ax, ay, az)
%   drawIMUAccelerationData(ax, ay, az, '../demo/image')
hold off;

% acc-X
figure('Name', 'Acceleration-X');
plot(accX);
title('Acceleration-X');
xlabel('sample index');
ylabel('m/s^2');
if (nargin >= 4)
    saveas(gcf, strcat(savePath, '/acc-x.svg'));
end

% acc-Y
figure('Name', 'Acceleration-Y');
plot(accY);
title('Acceleration-Y');
xlabel('sample index');
ylabel('m/s^2');
if (nargin >= 4)
    saveas(gcf, strcat(savePath, '/acc-y.svg'));
end

% acc-Z
figure('Name', 'Acceleration-Z');
plot(accZ);
title('Acceleration-Z');
xlabel('sample index');
ylabel('m/s^2');
if (nargin >= 4)
    saveas(gcf, strcat(savePath, '/acc-z.svg'));
end
end

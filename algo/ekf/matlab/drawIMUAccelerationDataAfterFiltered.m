function drawIMUAccelerationDataAfterFiltered(accX, accY, accZ, H, savePath)
% filter IMU acceleration data by coefficient 'H', then draw results, 
%   the pictures will be saved as 'savePath/acc-xf.svg'
%   & 'savePath/acc-yf.svg' & 'savePath/acc-zf.svg'
%   if 'savePath' passed
%
% input: acceleration-X (m/s^2)
%        acceleration-Y (m/s^2)
%        acceleration-Z (m/s^2)
%        filter coefficient
%        path where the pictures will be saved
% 
% Example:
%   drawIMUAccelerationDataAfterFiltered(ax, ay, az, H)
%   drawIMUAccelerationDataAfterFiltered(ax, ay, az, H, '../demo/image')
hold off;

% acc-X fir-filtered
accXF = conv(accX, H, "same");
figure('Name', 'AccelerationFiltered-X');
plot(accXF);
title('AccelerationFiltered-X');
xlabel('sample index');
ylabel('m/s^2');
if (nargin >= 5)
    saveas(gcf, strcat(savePath, '/acc-xf.svg'));
end

% acc-Y fir-filtered
accYF = conv(accY, H, "same");
figure('Name', 'AccelerationFiltered-Y');
plot(accYF);
title('AccelerationFiltered-Y');
xlabel('sample index');
ylabel('m/s^2');
if (nargin >= 5)
    saveas(gcf, strcat(savePath, '/acc-yf.svg'));
end

% acc-Z fir-filtered
accZF = conv(accZ, H, "same");
figure('Name', 'AccelerationFiltered-Z');
plot(accZF);
title('AccelerationFiltered-Z');
xlabel('sample index');
ylabel('m/s^2');
if (nargin >= 5)
    saveas(gcf, strcat(savePath, '/acc-zf.svg'));
end
end
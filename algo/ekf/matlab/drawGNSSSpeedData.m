function drawGNSSSpeedData(speedNorth, speedEast, speedUp, savePath)
% draw GNSS speed data, the pictures will be saved as
%   'savePath/speed-north.svg' & 'savePath/speed-east.svg'
%   & 'savePath/speed-up.svg' if 'savePath' passed
%
% input: speed-north (m/s)
%        speed-east (m/s)
%        speed-up (m/s)
%        path where the pictures will be saved
% 
% Example:
%   drawGNSSSpeedData(sn, se, su)
%   drawGNSSSpeedData(sn, se, su, '../demo/image')
hold off;

% speed north
figure('Name', 'Speed-North');
plot(speedNorth);
title('Speed North');
xlabel('sample index');
ylabel('m/s');
if (nargin >= 4)
    saveas(gcf, strcat(savePath, '/speed-north.svg'));
end

% speed east
figure('Name', 'Speed-East');
plot(speedEast);
title('Speed East');
xlabel('sample index');
ylabel('m/s');
if (nargin >= 4)
    saveas(gcf, strcat(savePath, '/speed-east.svg'));
end

% speed up
figure('Name', 'Speed-Up');
plot(speedUp);
title('Speed Up');
xlabel('sample index');
ylabel('m/s');
if (nargin >= 4)
    saveas(gcf, strcat(savePath, '/speed-up.svg'));
end
end
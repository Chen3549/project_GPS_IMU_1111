function drawGNSSTrack(latData, lonData, savePath)
% draw GNSS track, the picture will be saved as
%   'savePath/GNSS-Track.svg' if 'savePath' passed
%
% input: GNSS latitude (rad)
%        GNSS longitude (rad)
%        path where the picture will be saved
% 
% Example:
%   drawGNSSTrack(lat, lon)
%   drawGNSSTrack(lat, lon, '../demo/image')
hold off;

geoscatter(rad2deg(latData), rad2deg(lonData), 15, 'r', 'o','filled');
geobasemap satellite;
title('GNSS Track');
grid on;

if nargin >= 3
    saveas(gcf, strcat(savePath, '/GNSS-Track.svg'));
end
end

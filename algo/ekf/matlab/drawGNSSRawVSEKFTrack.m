function drawGNSSRawVSEKFTrack(latRaw, lonRaw, latEKF, lonEKF, savePath)
% draw GNSS raw track and EKF track, the picture will be saved as
%   'savePath/GNSS-Raw-VS-EKF-Track.svg' if 'savePath' passed
%
% input: raw GNSS latitude (rad)
%        raw GNSS longitude (rad)
%        EKF GNSS latitude (rad)
%        EKF GNSS longitude (rad)
%        path where the picture will be saved
% 
% Example:
%   drawGNSSRawVSEKFTrack(lat, lon, latf, lonf)
%   drawGNSSRawVSEKFTrack(lat, lon, latf, lonf, '../demo/image')
hold off;

% draw GNSS raw track
geoscatter(rad2deg(latRaw), rad2deg(lonRaw), 15, 'r', 'o','filled','DisplayName', 'GNSS raw track');
geobasemap satellite;
hold on;

% draw GNSS EKF track
geoscatter(rad2deg(latEKF), rad2deg(lonEKF), 15, 'b', 'o','filled','DisplayName', 'GNSS filtered track');

title('GNSS Raw VS EKF Track');
legend('Location', 'northeast');
grid on;

if nargin >= 5
    saveas(gcf, strcat(savePath, '/GNSS-Raw-VS-EKF-Track.svg'));
end
end
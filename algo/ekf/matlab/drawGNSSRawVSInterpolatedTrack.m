function drawGNSSRawVSInterpolatedTrack(latRaw, lonRaw, latinterpolated, loninterpolated, savePath)
% draw GNSS raw track and interpolated track, the picture will be saved as
%   'savePath/GNSS-Raw-VS-Interpolated-Track.svg' if 'savePath' passed
%
% input: raw GNSS latitude (rad)
%        raw GNSS longitude (rad)
%        GNSS latitude interpolated (rad)
%        GNSS longitude interpolated (rad)
%        path where the picture will be saved
% 
% Example:
%   drawGNSSRawVSInterpolatedTrack(lat, lon, lati, loni)
%   drawGNSSRawVSInterpolatedTrack(lat, lon, lati, loni, '../demo/image')
hold off;

% draw GNSS raw track
geoscatter(rad2deg(latRaw), rad2deg(lonRaw), 15, 'r', 'o','filled','DisplayName', 'GNSS raw track');
geobasemap satellite;
hold on;

% draw GNSS interpolated track
geoscatter(rad2deg(latinterpolated), rad2deg(loninterpolated), 15, 'b', 'o','filled','DisplayName', 'GNSS interpolated track');

title('GNSS Raw VS Interpolated Track');
legend('Location', 'northeast');
grid on;

if nargin >= 5
    saveas(gcf, strcat(savePath, '/GNSS-Raw-VS-Interpolated-Track.svg'));
end
end
function [dataGPS] = extractDataGNSS(dataGNSSPath)
%EXTRACTDATAGNSS
% input: data GNSS path
%
% output:
% data matrix GNSS:
% dataGPS format: time(UTC, μs) / latitude(rad) / longitude(rad) / altitude(m) /
%                 speed North(m/s) / speed East(m/s) / speed Up(m/s)
%

% read raw data csv
% NEMA time lat(degree) lon(degree) msl(m) kph(km/h) cogt(degree)
rawDataGPS = readtable(dataGNSSPath);
rawDataGPS.lat_rad = deg2rad(rawDataGPS.lat); % degree to rad
rawDataGPS.lon_rad = deg2rad(rawDataGPS.lon); % degree to rad
rawDataGPS.mps = rawDataGPS.kph / 3.6; % km/h to m/s
rawDataGPS.cogt_rad = deg2rad(rawDataGPS.cogt); % degree to rad
% resolution of speed
rawDataGPS.speed_north = rawDataGPS.mps .* cos(rawDataGPS.cogt_rad);
rawDataGPS.speed_east = rawDataGPS.mps .* sin(rawDataGPS.cogt_rad);
rawDataGPS.speed_up = zeros(size(rawDataGPS.time)); % can't get from GNSS

dataGPS = table2array(rawDataGPS(:, {'time', 'lat_rad', 'lon_rad', 'msl', 'speed_north', 'speed_east', 'speed_up'}));
end


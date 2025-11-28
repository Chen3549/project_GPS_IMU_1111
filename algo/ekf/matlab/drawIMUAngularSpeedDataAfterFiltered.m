function drawIMUAngularSpeedDataAfterFiltered(angSpeedX, angSpeedY, angSpeedZ, H, savePath)
% filter IMU angular-speed data by coefficient 'H', then draw results, 
%   the pictures will be saved as 'savePath/angSpeed-xf.svg'
%   & 'savePath/angSpeed-yf.svg' & 'savePath/angSpeed-zf.svg'
%   if 'savePath' passed
%
% input: angular-speed X (rad/s)
%        angular-speed Y (rad/s)
%        angular-speed Z (rad/s)
%        filter coefficient
%        path where the pictures will be saved
% 
% Example:
%   drawIMUAngularSpeedDataAfterFiltered(asx, asy, asz, H)
%   drawIMUAngularSpeedDataAfterFiltered(asx, asy, asz, H, '../demo/image')
hold off;

% angular-speed X fir-filtered
angSpeedXF = conv(angSpeedX, H, "same");
figure('Name', 'AngularSpeedFiltered-X');
plot(angSpeedXF);
title('AngularSpeedFiltered-X');
xlabel('sample index');
ylabel('rad/s');
if (nargin >= 5)
    saveas(gcf, strcat(savePath, '/angSpeed-xf.svg'));
end

% angular-speed Y fir-filtered
angSpeedYF = conv(angSpeedY, H, "same");
figure('Name', 'AngularSpeedFiltered-Y');
plot(angSpeedYF);
title('AngularSpeedFiltered-Y');
xlabel('sample index');
ylabel('rad/s');
if (nargin >= 5)
    saveas(gcf, strcat(savePath, '/angSpeed-yf.svg'));
end

% angular-speed Z fir-filtered
angSpeedZF = conv(angSpeedZ, H, "same");
figure('Name', 'AngularSpeedFiltered-Z');
plot(angSpeedZF);
title('AngularSpeedFiltered-Z');
xlabel('sample index');
ylabel('rad/s');
if (nargin >= 5)
    saveas(gcf, strcat(savePath, '/angSpeed-zf.svg'));
end
end
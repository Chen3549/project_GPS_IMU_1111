function drawIMUAngularSpeedData(angSpeedX, angSpeedY, angSpeedZ, savePath)
% draw IMU angular-speed data, the pictures will be saved as
%   'savePath/angSpeed-x.svg' & 'savePath/angSpeed-y.svg'
%   & 'savePath/angSpeed-z.svg' if 'savePath' passed
%
% input: angular-speed X (rad/s)
%        angular-speed Y (rad/s)
%        angular-speed Z (rad/s)
%        path where the pictures will be saved
% 
% Example:
%   drawIMUAngularSpeedData(asx, asy, asz)
%   drawIMUAngularSpeedData(asx, asy, asz, '../demo/image')
hold off;

% angular-speed X
figure("Name", 'AngularSpeed-X');
plot(angSpeedX);
title('AngularSpeed-X');
xlabel('sample index');
ylabel('rad/s');
if (nargin >= 4)
    saveas(gcf, strcat(savePath, '/angSpeed-x.svg'));
end

% angular-speed Y
figure("Name", 'AngularSpeed-Y');
plot(angSpeedY);
title('AngularSpeed-Y');
xlabel('sample index');
ylabel('rad/s');
if (nargin >= 4)
    saveas(gcf, strcat(savePath, '/angSpeed-y.svg'));
end

% angular-speed Z
figure("Name", 'AngularSpeed-Z');
plot(angSpeedZ);
title('AngularSpeed-Z');
xlabel('sample index');
ylabel('rad/s');
if (nargin >= 4)
    saveas(gcf, strcat(savePath, '/angSpeed-z.svg'));
end
end
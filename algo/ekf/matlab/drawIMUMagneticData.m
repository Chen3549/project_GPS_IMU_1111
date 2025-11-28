function drawIMUMagneticData(magX, magY, magZ, savePath)
% draw IMU magnetic data, the pictures will be saved as
%   'savePath/magnetic-x.svg' & 'savePath/magnetic-y.svg'
%   & 'savePath/magnetic-z.svg' if 'savePath' passed
%
% input: magnetic-X (μT)
%        magnetic-Y (μT)
%        magnetic-Z (μT)
%        path where the pictures will be saved
% 
% Example:
%   drawIMUMagneticData(mx, my, mz)
%   drawIMUMagneticData(mx, my, mz, '../demo/image')
hold off;

% magnetic X
figure("Name", 'Magnetic-X');
plot(magX);
title('Magnetic-X');
xlabel('sample index');
ylabel('μT');
if (isSave)
    saveas(gcf, strcat(savePath, '/magnetic-x.svg'));
end

% magnetic Y
figure("Name", 'Magnetic-Y');
plot(magY);
title('Magnetic-Y');
xlabel('sample index');
ylabel('μT');
if (isSave)
    saveas(gcf, strcat(savePath, '/magnetic-y.svg'));
end

% magnetic Z
figure("Name", 'Magnetic-Z');
plot(magZ);
title('Magnetic-Z');
xlabel('sample index');
ylabel('μT');
if (isSave)
    saveas(gcf, strcat(savePath, '/magnetic-z.svg'));
end
end
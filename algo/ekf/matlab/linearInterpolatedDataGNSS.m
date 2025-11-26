function [dataGNSSInterpolated, dataIMUInterpolated] = linearInterpolatedDataGNSS(dataGNSS, dataIMU)
% linear interpolated GNSS data
% input: data GNSS
%        data IMU
% output:
% data GNSS interpolated
% data IMU interpolated
%
dataGNSSInterpolated = zeros(height(dataIMU), length(dataGNSS));
dataIMUInterpolated = zeros(size(dataIMU));

interp_cnt = 0;
j = 1; % GNSS data iterate
for i = 1:height(dataIMU) % IMU data iterate
    % if j > height(dataGNSS)
    %     break;
    % end
    % if dataGNSS(j, 1) == dataIMU(i, 1)
    %     interp_cnt = interp_cnt + 1;
    %     dataGNSSInterpolated(interp_cnt, :) = dataGNSS(j, :);
    %     dataIMUInterpolated(interp_cnt, :) = dataIMU(i, :);
    % else
    %     while j < height(dataGNSS) && dataGNSS(j, 1) < dataIMU(i, 1)
    %         if dataGNSS(j + 1, 1) > dataIMU(i, 1)
    %             interp_cnt = interp_cnt + 1;
    %             % interpolate GNSS
    %             k = (dataIMU(i, 1) - dataGNSS(j, 1)) / (dataGNSS(j + 1, 1) - dataGNSS(j, 1));
    %             dataGNSSInterpolated(interp_cnt, 1) = dataIMU(i, 1);
    %             dataGNSSInterpolated(interp_cnt, 2:end) = dataGNSS(j, 2:end) + k * (dataGNSS(j + 1, 2:end) - dataGNSS(j, 2:end));
    %             dataIMUInterpolated(interp_cnt, :) = dataIMU(i, :);
    %             break;
    %         else
    %             j = j + 1;
    %         end
    %     end
    % end



    while j <= height(dataGNSS)
        if dataGNSS(j, 1) > dataIMU(i, 1)
            break;
        elseif dataGNSS(j, 1) == dataIMU(i, 1)
            interp_cnt = interp_cnt + 1;
            dataGNSSInterpolated(interp_cnt, :) = dataGNSS(j, :);
            dataIMUInterpolated(interp_cnt, :) = dataIMU(i, :);
            break;
        else % dataGNSS(j, 1) < dataIMU(i, 1) 
            if j < height(dataGNSS) && dataGNSS(j + 1, 1) > dataIMU(i, 1)
                interp_cnt = interp_cnt + 1;
                % interpolate GNSS
                k = (dataIMU(i, 1) - dataGNSS(j, 1)) / (dataGNSS(j + 1, 1) - dataGNSS(j, 1));
                dataGNSSInterpolated(interp_cnt, 1) = dataIMU(i, 1);
                dataGNSSInterpolated(interp_cnt, 2:end) = dataGNSS(j, 2:end) + k * (dataGNSS(j + 1, 2:end) - dataGNSS(j, 2:end));
                dataIMUInterpolated(interp_cnt, :) = dataIMU(i, :);
                break;
            else
                j = j + 1;
            end
        end
    end
end

dataGNSSInterpolated = dataGNSSInterpolated(1:interp_cnt, :);
dataIMUInterpolated = dataIMUInterpolated(1:interp_cnt, :);
end
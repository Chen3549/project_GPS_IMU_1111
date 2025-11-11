function draw_ekf_data(dataPath, isSave)
    % read csv
    df = readtable(dataPath);

    % display
    head(df)
    height(df)

    % get GNSS raw data and filtered data
    df_raw = df(:, 4:22);%df(:, 2:6);
    df_filtered = df(:, 23:27);%df(:, 7:11);

    % get bounding box
    bbox = [min(df_raw.longitude), max(df_raw.longitude), ...
        min(df_raw.latitude), max(df_raw.latitude)];
    disp(bbox)

    % create figure
    figure('OuterPosition', [0, 0, 1080, 1080]);
    hold on;

    % set coordinate axis range
    xlim([bbox(1), bbox(2)]);
    ylim([bbox(3), bbox(4)]);

    % format axis
    ax = gca;
    ax.XAxis.Exponent = 0;
    ax.YAxis.Exponent = 0;
    xtickformat('%.7f');
    ytickformat('%.7f');
    
    % draw raw data
    x_r = df_raw.longitude;
    y_r = df_raw.latitude;
    yaw_r = df_raw.yaw;
    pitch_r = df_raw.pitch;
    roll_r = df_raw.roll;

    % raw data start and end point
    plot(x_r(1), y_r(1), 'yo', 'MarkerSize', 15, 'MarkerFaceColor', 'y', 'HandleVisibility', 'off');
    plot(x_r(end), y_r(end), 'ro', 'MarkerSize', 15, 'MarkerFaceColor', 'r', 'HandleVisibility', 'off');

    % raw data scatter
    scatter(x_r, y_r, 30, 'b', 'filled', 'MarkerFaceAlpha', 0.8, ...
        'DisplayName', 'GNSS Raw data');

    % raw data directional arraw
    %quiver(x_r, y_r, cos(yaw_r), sin(yaw_r), 0.5, 'k', 'LineWidth', 0.5, 'MaxHeadSize', 0.5, 'HandleVisibility', 'off');

    % draw filtered data
    x_f = df_filtered.filtered_longitude;
    y_f = df_filtered.filtered_latitude;
    yaw_f = df_filtered.filtered_yaw;
    pitch_f = df_filtered.filtered_pitch;
    roll_f = df_filtered.filtered_roll;

    % filtered data start and end point
    plot(x_f(1), y_f(1), 'yo', 'MarkerSize', 15, 'MarkerFaceColor', 'y', 'HandleVisibility', 'off');
    plot(x_f(end), y_f(end), 'ro', 'MarkerSize', 15, 'MarkerFaceColor', 'r', 'HandleVisibility', 'off');

    % filtered data scatter
    scatter(x_f, y_f, 30, 'g', 'filled', 'MarkerFaceAlpha', 0.8, ...
        'DisplayName', 'EKF Filtered GNSS');

    % filtered data directional arraw
    %quiver(x_f, y_f, cos(yaw_f), sin(yaw_f), 0.5, 'k', 'LineWidth', 0.5, 'MaxHeadSize', 0.5, 'HandleVisibility', 'off');

    % add title and legend
    title('GNSS Raw (Vs) EKF Filtered GNSS');
    legend('Location', 'northwest');
    grid on;
    hold off;

    if (isSave)
        saveas(gcf, './test/raw_vs_ekf.svg');
    end
end

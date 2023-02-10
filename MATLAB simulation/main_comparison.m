%% データ読み込み
close all
clear

global traffic_settings

graphics = Graphics(traffic_settings);

%13秒に12台の場合
load('12 vehicles per 12 seconds')
previous_time_series = graphics.remove_data_out_of_range(previous_time_series);
proposed_time_series = graphics.remove_data_out_of_range(proposed_time_series);
previous_fuel_consumption_12 = graphics.calculate_fuel_consumption(previous_time_series);
proposed_fuel_consumption_12 = graphics.calculate_fuel_consumption(proposed_time_series);
previous_average_velocity_12 = graphics.calculate_average_velocity(previous_time_series);
proposed_average_velocity_12 = graphics.calculate_average_velocity(proposed_time_series);
previous_ave_travel_time_12 = graphics.calculate_ave_travel_time(previous_time_series);
proposed_ave_travel_time_12 = graphics.calculate_ave_travel_time(proposed_time_series);
previous_std_travel_time_12 = graphics.calculate_std_travel_time(previous_time_series);
proposed_std_travel_time_12 = graphics.calculate_std_travel_time(proposed_time_series);
previous_ave_waiting_time_12 = graphics.calculate_ave_waiting_time(previous_time_series);
proposed_ave_waiting_time_12 = graphics.calculate_ave_waiting_time(proposed_time_series);
previous_std_waiting_time_12 = graphics.calculate_std_waiting_time(previous_time_series);
proposed_std_waiting_time_12 = graphics.calculate_std_waiting_time(proposed_time_series);

load('12 vehicles per 13 seconds')
previous_time_series = graphics.remove_data_out_of_range(previous_time_series);
proposed_time_series = graphics.remove_data_out_of_range(proposed_time_series);
previous_fuel_consumption_13 = graphics.calculate_fuel_consumption(previous_time_series);
proposed_fuel_consumption_13 = graphics.calculate_fuel_consumption(proposed_time_series);
previous_average_velocity_13 = graphics.calculate_average_velocity(previous_time_series);
proposed_average_velocity_13 = graphics.calculate_average_velocity(proposed_time_series);
previous_ave_travel_time_13 = graphics.calculate_ave_travel_time(previous_time_series);
proposed_ave_travel_time_13 = graphics.calculate_ave_travel_time(proposed_time_series);
previous_std_travel_time_13 = graphics.calculate_std_travel_time(previous_time_series);
proposed_std_travel_time_13 = graphics.calculate_std_travel_time(proposed_time_series);
previous_ave_waiting_time_13 = graphics.calculate_ave_waiting_time(previous_time_series);
proposed_ave_waiting_time_13 = graphics.calculate_ave_waiting_time(proposed_time_series);
previous_std_waiting_time_13 = graphics.calculate_std_waiting_time(previous_time_series);
proposed_std_waiting_time_13 = graphics.calculate_std_waiting_time(proposed_time_series);

load('12 vehicles per 15 seconds')
previous_time_series = graphics.remove_data_out_of_range(previous_time_series);
proposed_time_series = graphics.remove_data_out_of_range(proposed_time_series);
previous_fuel_consumption_15 = graphics.calculate_fuel_consumption(previous_time_series);
proposed_fuel_consumption_15 = graphics.calculate_fuel_consumption(proposed_time_series);
previous_average_velocity_15 = graphics.calculate_average_velocity(previous_time_series);
proposed_average_velocity_15 = graphics.calculate_average_velocity(proposed_time_series);
previous_ave_travel_time_15 = graphics.calculate_ave_travel_time(previous_time_series);
proposed_ave_travel_time_15 = graphics.calculate_ave_travel_time(proposed_time_series);
previous_std_travel_time_15 = graphics.calculate_std_travel_time(previous_time_series);
proposed_std_travel_time_15 = graphics.calculate_std_travel_time(proposed_time_series);
previous_ave_waiting_time_15 = graphics.calculate_ave_waiting_time(previous_time_series);
proposed_ave_waiting_time_15 = graphics.calculate_ave_waiting_time(proposed_time_series);
previous_std_waiting_time_15 = graphics.calculate_std_waiting_time(previous_time_series);
proposed_std_waiting_time_15 = graphics.calculate_std_waiting_time(proposed_time_series);

load('12 vehicles per 17 seconds')
previous_time_series = graphics.remove_data_out_of_range(previous_time_series);
proposed_time_series = graphics.remove_data_out_of_range(proposed_time_series);
previous_fuel_consumption_17 = graphics.calculate_fuel_consumption(previous_time_series);
proposed_fuel_consumption_17 = graphics.calculate_fuel_consumption(proposed_time_series);
previous_average_velocity_17 = graphics.calculate_average_velocity(previous_time_series);
proposed_average_velocity_17 = graphics.calculate_average_velocity(proposed_time_series);
previous_ave_travel_time_17 = graphics.calculate_ave_travel_time(previous_time_series);
proposed_ave_travel_time_17 = graphics.calculate_ave_travel_time(proposed_time_series);
previous_std_travel_time_17 = graphics.calculate_std_travel_time(previous_time_series);
proposed_std_travel_time_17 = graphics.calculate_std_travel_time(proposed_time_series);
previous_ave_waiting_time_17 = graphics.calculate_ave_waiting_time(previous_time_series);
proposed_ave_waiting_time_17 = graphics.calculate_ave_waiting_time(proposed_time_series);
previous_std_waiting_time_17 = graphics.calculate_std_waiting_time(previous_time_series);
proposed_std_waiting_time_17 = graphics.calculate_std_waiting_time(proposed_time_series);

load('12 vehicles per 20 seconds')
previous_time_series = graphics.remove_data_out_of_range(previous_time_series);
proposed_time_series = graphics.remove_data_out_of_range(proposed_time_series);
previous_fuel_consumption_20 = graphics.calculate_fuel_consumption(previous_time_series);
proposed_fuel_consumption_20 = graphics.calculate_fuel_consumption(proposed_time_series);
previous_average_velocity_20 = graphics.calculate_average_velocity(previous_time_series);
proposed_average_velocity_20 = graphics.calculate_average_velocity(proposed_time_series);
previous_ave_travel_time_20 = graphics.calculate_ave_travel_time(previous_time_series);
proposed_ave_travel_time_20 = graphics.calculate_ave_travel_time(proposed_time_series);
previous_std_travel_time_20 = graphics.calculate_std_travel_time(previous_time_series);
proposed_std_travel_time_20 = graphics.calculate_std_travel_time(proposed_time_series);
previous_ave_waiting_time_20 = graphics.calculate_ave_waiting_time(previous_time_series);
proposed_ave_waiting_time_20 = graphics.calculate_ave_waiting_time(proposed_time_series);
previous_std_waiting_time_20 = graphics.calculate_std_waiting_time(previous_time_series);
proposed_std_waiting_time_20 = graphics.calculate_std_waiting_time(proposed_time_series);

%% グラフにする
% 燃料消費量
figure('Position', [100 100 400 240])% [left bottom width height]
%traffic_volume = 12./[12 40/3 15 103/6 20]*3600;
traffic_volume = 12./[20 103/6 15 40/3 12]*3600;
fuel_consumption = zeros(5, 2);
fuel_consumption(1, :) = [previous_fuel_consumption_20 proposed_fuel_consumption_20];
fuel_consumption(2, :) = [previous_fuel_consumption_17 proposed_fuel_consumption_17];
fuel_consumption(3, :) = [previous_fuel_consumption_15 proposed_fuel_consumption_15];
fuel_consumption(4, :) = [previous_fuel_consumption_13 proposed_fuel_consumption_13];
fuel_consumption(5, :) = [previous_fuel_consumption_12 proposed_fuel_consumption_12];

b = bar(traffic_volume, fuel_consumption);
ylim([0.04 0.064])
legend("Traditional traffic", "Proposed system", 'Location','northwest')

% 平均速度
figure('Position', [100 100 400 240]) % [left bottom width height]
average_velocity = zeros(5, 2);
average_velocity(1, :) = [previous_average_velocity_20 proposed_average_velocity_20];
average_velocity(2, :) = [previous_average_velocity_17 proposed_average_velocity_17];
average_velocity(3, :) = [previous_average_velocity_15 proposed_average_velocity_15];
average_velocity(4, :) = [previous_average_velocity_13 proposed_average_velocity_13];
average_velocity(5, :) = [previous_average_velocity_12 proposed_average_velocity_12];

b = bar(traffic_volume, average_velocity);
legend("Traditional traffic", "Proposed system")

%移動時間
figure('Position', [100 100 400 240]) % [left bottom width height]
travel_time = zeros(5, 2);
travel_time(1, :) = [previous_ave_travel_time_20 proposed_ave_travel_time_20];
travel_time(2, :) = [previous_ave_travel_time_17 proposed_ave_travel_time_17];
travel_time(3, :) = [previous_ave_travel_time_15 proposed_ave_travel_time_15];
travel_time(4, :) = [previous_ave_travel_time_13 proposed_ave_travel_time_13];
travel_time(5, :) = [previous_ave_travel_time_12 proposed_ave_travel_time_12];

travel_time_err = zeros(5, 2);
travel_time_err(1, :) = [previous_std_travel_time_20 proposed_std_travel_time_20];
travel_time_err(2, :) = [previous_std_travel_time_17 proposed_std_travel_time_17];
travel_time_err(3, :) = [previous_std_travel_time_15 proposed_std_travel_time_15];
travel_time_err(4, :) = [previous_std_travel_time_13 proposed_std_travel_time_13];
travel_time_err(5, :) = [previous_std_travel_time_12 proposed_std_travel_time_12];

hb = bar(traffic_volume, travel_time);
hold on;
for k = 1:2
    % get x position per group
    xpos = hb(k).XData + hb(k).XOffset;
    % draw errorbar
    errorbar(xpos, travel_time(:, k), travel_time_err(:, k), 'LineStyle', 'none', 'Color', 'k', 'LineWidth', 1);
end
%待ち時間
figure('Position', [100 100 400 240]) % [left bottom width height]
waiting_time = zeros(5, 2);
waiting_time(1, :) = [previous_ave_waiting_time_20 proposed_ave_waiting_time_20]; 
waiting_time(2, :) = [previous_ave_waiting_time_17 proposed_ave_waiting_time_17];
waiting_time(3, :) = [previous_ave_waiting_time_15 proposed_ave_waiting_time_15];
waiting_time(4, :) = [previous_ave_waiting_time_13 proposed_ave_waiting_time_13];
waiting_time(5, :) = [previous_ave_waiting_time_12 proposed_ave_waiting_time_12];

waiting_time_err = zeros(5, 2);
waiting_time_err(1, :) = [previous_std_waiting_time_20 proposed_std_waiting_time_20];
waiting_time_err(2, :) = [previous_std_waiting_time_17 proposed_std_waiting_time_17];
waiting_time_err(3, :) = [previous_std_waiting_time_15 proposed_std_waiting_time_15];
waiting_time_err(4, :) = [previous_std_waiting_time_13 proposed_std_waiting_time_13];
waiting_time_err(5, :) = [previous_std_waiting_time_12 proposed_std_waiting_time_12];

hb = bar(traffic_volume, waiting_time);
hold on;
for k = 1:2
    % get x position per group
    xpos = hb(k).XData + hb(k).XOffset;
    % draw errorbar
    errorbar(xpos, waiting_time(:, k), waiting_time_err(:, k), 'LineStyle', 'none', 'Color', 'k', 'LineWidth', 1);
end

%{
table = [travel_time travel_time_err waiting_time waiting_time_err];
filename = 'travel time and waiting time.xlsm';
writematrix(table, filename, 'Sheet', 1, 'Range', 'E3:L8')
%}

%% データ読み込み
close all
clear

global traffic_settings

G = Graphics(traffic_settings);

%40秒に12台の場合
load('12 vehicles per 40 seconds')
[previous, proposed] = G.remove_data_out_of_range(previous_time_series, proposed_time_series);
[previous_fuel_consumption_40, proposed_fuel_consumption_40] = G.calculate_fuel_consumption(previous, proposed);
[previous_average_velocity_40, proposed_average_velocity_40] = G.calculate_average_velocity(previous, proposed);

%45秒に12台の場合
load('12 vehicles per 45 seconds')
[previous, proposed] = G.remove_data_out_of_range(previous_time_series, proposed_time_series);
[previous_fuel_consumption_45, proposed_fuel_consumption_45] = G.calculate_fuel_consumption(previous, proposed);
[previous_average_velocity_45, proposed_average_velocity_45] = G.calculate_average_velocity(previous, proposed);

%50秒に12台の場合
load('12 vehicles per 50 seconds')
[previous, proposed] = G.remove_data_out_of_range(previous_time_series, proposed_time_series);
[previous_fuel_consumption_50, proposed_fuel_consumption_50] = G.calculate_fuel_consumption(previous, proposed);
[previous_average_velocity_50, proposed_average_velocity_50] = G.calculate_average_velocity(previous, proposed);

%55秒に12台の場合
load('12 vehicles per 55 seconds')
[previous, proposed] = G.remove_data_out_of_range(previous_time_series, proposed_time_series);
[previous_fuel_consumption_55, proposed_fuel_consumption_55] = G.calculate_fuel_consumption(previous, proposed);
[previous_average_velocity_55, proposed_average_velocity_55] = G.calculate_average_velocity(previous, proposed);

%% グラフにする
% 燃料消費量
figure
x = [40 45 50 55];
y = zeros(4, 2);
y(1, :) = [previous_fuel_consumption_40 proposed_fuel_consumption_40];
y(2, :) = [previous_fuel_consumption_45 proposed_fuel_consumption_45];
y(3, :) = [previous_fuel_consumption_50 proposed_fuel_consumption_50];
y(4, :) = [previous_fuel_consumption_55 proposed_fuel_consumption_55];
bar(x, y)
ylim([40 60])
xlabel('traffic volume [s/12 vehicles]')
ylabel('total fuel consumption [l]')

% 平均速度
figure
x = [40 45 50 55];
y = zeros(4, 2);
y(1, :) = [previous_average_velocity_40 proposed_average_velocity_40];
y(2, :) = [previous_average_velocity_45 proposed_average_velocity_45];
y(3, :) = [previous_average_velocity_50 proposed_average_velocity_50];
y(4, :) = [previous_average_velocity_55 proposed_average_velocity_55];
bar(x, y)
xlabel('traffic volume [s/12 vehicles]')
ylabel('average velocity [km/h]')
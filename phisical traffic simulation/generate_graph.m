%% 得られたデータの整理
close all
clear

global traffic_settings

load('12 vehicles per 55 seconds')

%人間モデルをグラフ化する
simulation_time = size(previous_time_series, 1);
vehicles = size(previous_time_series, 2);
t = traffic_settings.dt*[0:simulation_time-1]'.*ones(simulation_time, vehicles);
repetitions = size(previous_time_series, 4);

% 範囲外の車の情報を除く
graphics = Graphics(traffic_settings);
[previous, proposed] = graphics.remove_data_out_of_range(previous_time_series, proposed_time_series);

%位置をグラフに表示する
for r = 1:repetitions
    graphics.graph_position(previous, t, r)

    %速度をグラフに表示する
    %graphics.graph_velocity(previous, t, r)
    
    %加速度をグラフに表示する
    %graphics.graph_accel(previous, t, r)
end

%MPCモデルをグラフ化する
simulation_time = size(proposed_time_series, 1);
vehicles = size(proposed_time_series, 2);
t = traffic_settings.dt*[0:simulation_time-1]'.*ones(simulation_time, vehicles);
repetitions = size(proposed_time_series, 4);
%位置をグラフに表示する
for r = 1:repetitions
    %位置をグラフに表示する
    graphics.graph_position(proposed, t, r) 
    
    %速度をグラフに表示する
    %graphics.graph_velocity(proposed, t, r)
    
    %加速度をグラフに表示する
    %graphics.graph_accel(proposed, t, r)
end

%燃料消費量の計算
[previous_fuel_consumption proposed_fuel_consumption] = graphics.calculate_fuel_consumption(previous, proposed);

%平均速度の計算(時速)
[previous_average_velocity proposed_average_velocity] = graphics.calculate_average_velocity(previous, proposed);


%% グラフにする
X = categorical({'Previous method','Proposed method'});
X = reordercats(X,{'Previous method','Proposed method'});

figure
Y = [previous_fuel_consumption proposed_fuel_consumption];
bar(X, Y)
ylabel('total fuel consumption [l]')

figure
Y = [previous_average_velocity proposed_average_velocity];
bar(X, Y)
ylabel('average velocity [km/h]')


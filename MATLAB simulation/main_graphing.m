%% 得られたデータの整理
close all
clear

global traffic_settings

load('12 vehicles per 20 seconds 2')

%人間モデルをグラフ化する
simulation_time = size(previous_time_series, 1);
vehicles = size(previous_time_series, 2);
previous_time = traffic_settings.dt*[0:simulation_time-1]'.*ones(simulation_time, vehicles);
repetitions = size(previous_time_series, 4);

% 範囲外の車の情報を除く
graphics = Graphics(traffic_settings);
previous_time_series = graphics.remove_data_out_of_range(previous_time_series);
proposed_time_series = graphics.remove_data_out_of_range(proposed_time_series);

%位置をグラフに表示する
for r = 1:repetitions
    graphics.graph_position(previous_time_series, previous_time, r);

    %速度をグラフに表示する
    %graphics.graph_velocity(previous_time_series, previous_time, r);
    
    %加速度をグラフに表示する
    %graphics.graph_accel(previous_time_series, previous_time, r);
end


%MPCモデルをグラフ化する
simulation_time = size(proposed_time_series, 1);
vehicles = size(proposed_time_series, 2);
proposed_time = traffic_settings.dt*[0:simulation_time-1]'.*ones(simulation_time, vehicles);
repetitions = size(proposed_time_series, 4);


%位置をグラフに表示する
for r = 1:repetitions
    %位置をグラフに表示する
    graphics.graph_position(proposed_time_series, proposed_time, r);
    %速度をグラフに表示する
    %graphics.graph_velocity(proposed_time_series, proposed_time, r);
    
    %加速度をグラフに表示する
    %graphics.graph_accel(proposed_time_series, proposed_time, r);
end


%燃料消費量の計算
previous_fuel_consumption = graphics.calculate_fuel_consumption(previous_time_series);
proposed_fuel_consumption = graphics.calculate_fuel_consumption(proposed_time_series);

%平均速度の計算(時速)
previous_average_velocity = graphics.calculate_average_velocity(previous_time_series);
proposed_average_velocity = graphics.calculate_average_velocity(proposed_time_series);

X = categorical({'Previous method','Proposed method'});
X = reordercats(X,{'Previous method','Proposed method'});
%{
%燃料消費量の比較
figure
Y = [previous_fuel_consumption proposed_fuel_consumption];
bar(X, Y)
ylabel('total fuel consumption [l]')

%平均速度の比較
figure
Y = [previous_average_velocity proposed_average_velocity];
bar(X, Y)
ylabel('average velocity [km/h]')
%}

%% 

R = 2;
graphics.graph_position(previous_time_series, previous_time, R);
graphics.graph_velocity(previous_time_series, previous_time, R);
graphics.graph_accel(previous_time_series, previous_time, R);

graphics.graph_position(proposed_time_series, proposed_time, R);
graphics.graph_velocity(proposed_time_series, proposed_time, R);
graphics.graph_accel(proposed_time_series, p roposed_time, R);

%速度分布のヒストグラム
graphics.graph_histogram(previous_time_series, R)
graphics.graph_histogram(proposed_time_series, R)

%データをエクセルに出力
previous_table = graphics.generate_xlsm_data(previous_time_series, R);
proposed_table = graphics.generate_xlsm_data(proposed_time_series, R);

previous_filename = 'traditional method.xlsm';
writematrix(previous_table, previous_filename, 'Sheet', 1, 'Range', 'A2')

proposed_filename = 'proposed method.xlsm';
writematrix(proposed_table, proposed_filename, 'Sheet', 1, 'Range', 'A2')
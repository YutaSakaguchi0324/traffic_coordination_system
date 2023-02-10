%% シミュレーション設定
close all
clear

global traffic_settings

%人間モデルのシミュレーション
%シミュレーション変数
traffic_settings.dt = 0.5; %シミュレーションの最小単位時間

%車の挿入についての変数
traffic_settings.inserted_vehicle_count = 12; %挿入する車の台数
traffic_settings.insert_times = 8;
traffic_settings.insert_time_interval = 40; %これにdtをかけた値になることに注意
%12s:24, 13s:26, 15s:30, 17s:34, 20s:40

%車変数
traffic_settings.vehicle_length = 5; %車の長さ
traffic_settings.gap_min = 2; %車の最小車間距離
traffic_settings.Vmax = 27; %最大速度
traffic_settings.Vdes = 23; %理想速度
traffic_settings.Vmin = 0; %最小速度
traffic_settings.Amax = 0.73; %最大加速度
traffic_settings.Ades = -1.67; %理想加速度
traffic_settings.Amin = -2; %最小加速度

%環境変数
traffic_settings.road_start = 0;
traffic_settings.road_length = 500;

%最適化に関する変数
traffic_settings.time_horizon = 3;
traffic_settings.optimize_interval = 1;
traffic_settings.group_vehicle_count = 4;
traffic_settings.alpha = 0.01;%0.001
traffic_settings.w1 = 0.1; %速度評価係数 0.1
traffic_settings.w2 = 1; %加速度評価係数 1
traffic_settings.w3 = 0.2; %リスク評価係数 0.3

traffic_settings.sp = -50; %12s:30 13s:33.75 15s:37 17s:42 19s:47 20s:50 21s:52 23s:58 25s:62 27s:67 30s:74
traffic_settings.er = -7;%-10

%その他の変数
traffic_settings.delta_v_threshold = 2;%2.6

%変数を決める
simulation_time = 1200;
R = rem(simulation_time, traffic_settings.optimize_interval);
simulation_time = simulation_time - R;
repetitions = 10;

%% ランダム配列を作る
PTS = PhysicalTrafficSimulation(traffic_settings);

random_array = PTS.generate_random_array(repetitions);
save('10 times random array', 'random_array')

load('10 times random array', 'random_array')
%% 人間交通モデル(従来の手法)
%データを記録する;
C = traffic_settings.optimize_interval;
S = simulation_time;
N = traffic_settings.inserted_vehicle_count;
T = traffic_settings.insert_times;
previous_time_series = NaN(6, N*T, S+1, repetitions);
tic
for r = 1:repetitions
    state_variable = [];
    count = 1;
    
    for t = 0:simulation_time
        %車列の挿入
        insert_information.t = t;
        insert_information.r = r;
        insert_information.count = count;
        insert_information.random_array = random_array;
        insert_information.state_variable = state_variable;
        
        [count, state_variable] = PTS.insert_vehicles(insert_information);
        
        %車を追従させる
        state_variable = PTS.follow_vehicles([], state_variable);
        
        %車情報を記録する
        previous_time_series(:, state_variable(1, :), t+1, r) = state_variable(2:7, :);
        
        %車線変更の実行
        state_variable = PTS.change_lanes(state_variable);
        
        %車を前の位置から並べ直す
        state_variable = PTS.sort_cars(state_variable);
        
        %車をシミュレーションから排除する
        state_variable = PTS.delete_vehicles(state_variable);
        
        %シミュレーションを終了する
        if size(state_variable, 2) == 0
            break
        end
        
    end
end

%% 最適交通モデル(提案された手法)

CTS = CyberTrafficSystem(traffic_settings);

%データを記録する
proposed_time_series = NaN(6, N*T, S+1, repetitions);

for r = 1:repetitions
    state_variable = [];
    count = 1;
    
    for t = 0:simulation_time
        if rem(t, traffic_settings.optimize_interval) ~= 0
            continue
        end
        
        %車列の挿入
        insert_information.t = t;
        insert_information.r = r;
        insert_information.count = count;
        insert_information.random_array = random_array;
        insert_information.state_variable = state_variable;
        
        [count, state_variable] = PTS.insert_vehicles(insert_information);
        
        %車両群をA,B,Cグループに分ける
        [A_state, B_state, C_state] = CTS.split_into_ABC(state_variable);
        
        %Aグループの処理
        [A_state_T_seconds, tail_state_T_seconds] = CTS.follow_vehicles_T_seconds([], A_state);
        
        %Bグループの処理
        [B_state_T_seconds, tail_state_T_seconds] = CTS.optimize_vehicles(tail_state_T_seconds, B_state);
        
        %Cグループの処理
        [C_state_T_seconds, tail_state_T_seconds] = CTS.follow_vehicles_T_seconds(tail_state_T_seconds, C_state);
        
        state_T_seconds = [A_state_T_seconds B_state_T_seconds C_state_T_seconds];
        
        %走行情報を記録する
        proposed_time_series(:, state_variable(1, :), t+2:t+C+1, r) = state_T_seconds(2:7, :, 2:C+1);
        state_variable = state_T_seconds(:, :, C+1);
        
        %車線変更の実行
        state_variable = PTS.change_lanes(state_variable);
        
        %車を前の位置から並べ直す
        state_variable = PTS.sort_cars(state_variable);
        
        %車をシミュレーションから排除する
        state_variable = PTS.delete_vehicles(state_variable);
        
        %シミュレーションを終了する
        if size(state_variable, 2) == 0
            break
        end
    end
end

toc
%% 結果を記録
previous_time_series = permute(previous_time_series, [3 2 1 4]);
proposed_time_series = permute(proposed_time_series, [3 2 1 4]);

save('12 vehicles per 20 seconds 2', 'previous_time_series', 'proposed_time_series');
%シミュレーション結果をグラフにする関数まとめ
classdef Graphics
    properties
        dt %単位時間
        road_length %道路の長さ
        delta_v_threshold %車が停止したとみなす速度
    end
    
    methods
        %コンストラクタ
        function obj = Graphics(init_value)
            obj.dt = init_value.dt;
            obj.road_length = init_value.road_length;
            obj.delta_v_threshold = init_value.delta_v_threshold;
        end
        
        %位置をグラフにする
        function graph_position(obj, time_series, t, r)
            lane_index = time_series(:, :, 4, r);
            right_vehicle_position = time_series(:, :, 1, r);
            right_vehicle_position(lane_index == 1) = NaN;
            left_vehicle_position = time_series(:, :, 1, r);
            left_vehicle_position(lane_index == 0) = NaN;
            
            figure('Position', [100 100 400 240])
            hold on;
            plot(t, time_series(:, :, 1, r), '-r')
            plot(t, right_vehicle_position, '-r')
            plot(t, left_vehicle_position, '-b')
            ylim([-100 obj.road_length+100])
            
            xlabel('time [s]')
            ylabel('position [m]')
        end
        
        %速度をグラフにする
        function graph_velocity(obj, time_series, t, r)
            lane_index = time_series(:, :, 4, r);
            left_vehicle_velocity = time_series(:, :, 2, r);
            left_vehicle_velocity(lane_index == 1) = NaN;
            right_vehicle_velocity = time_series(:, :, 2, r);
            right_vehicle_velocity(lane_index == 0) = NaN;
            
            figure('Position', [100 100 400 240])
            hold on;
            plot(t, time_series(:, :, 2, r), '-r')
            plot(t, left_vehicle_velocity, '-r')
            plot(t, right_vehicle_velocity, '-b')
            
            xlabel('time [s]')
            ylabel('velocity [km/h]')
        end
        
        %加速度をグラフにする
        function graph_accel(obj, time_series, t, r)
            lane_index = time_series(:, :, 4, r);
            left_vehicle_accel = time_series(:, :, 3, r);
            left_vehicle_accel(lane_index == 1) = NaN;
            right_vehicle_accel = time_series(:, :, 3, r);
            right_vehicle_accel(lane_index == 0) = NaN;
            
            figure('Position', [100 100 400 240])
            hold on;
            plot(t, time_series(:, :, 3, r), '-r')
            plot(t, left_vehicle_accel, '-r')
            plot(t, right_vehicle_accel, '-b')
            
            xlabel('time [s]')
            ylabel('acceleration [m/s^2]')
        end
        
        %速度のヒストグラム
        function graph_histogram(obj, time_series, r)
            velocity = time_series(:, :, 2, r);
            figure('Position', [100 100 400 240])
            histogram(velocity);
            xlabel('velocity [km/h]')
            ylabel('frequency [-]')
        end
        
        %道路外の車の情報を除く
        function t = remove_data_out_of_range(obj, time_series)
            position = time_series(:, :, 1, :);
            velocity = time_series(:, :, 2, :);
            accel = time_series(:, :, 3, :);
            lane = time_series(:, :, 4, :);
            lanechange = time_series(:, :, 5, :);
            cooldown = time_series(:, :, 6, :);
            
            invalid_index = (position <= -130 | obj.road_length + 130 <= position);
            position(invalid_index) = NaN;
            velocity(invalid_index) = NaN;
            accel(invalid_index) = NaN;
            lane(invalid_index) = NaN;
            anechange(invalid_index) = NaN;
            cooldown(invalid_index) = NaN;
            
            time_series(:, :, 1, :) = position;
            time_series(:, :, 2, :) = velocity;
            time_series(:, :, 3, :) = accel;
            time_series(:, :, 4, :) = lane;
            time_series(:, :, 5, :) = lanechange;
            time_series(:, :, 6, :) = cooldown;

            t = time_series;
        end
        
        %単位時間・単位距離あたりの一台の燃料消費量の計算
        function Fml = FuelConsumptionGF(obj, position, velocity, accel)
            velocity = velocity(2:end, :, :, :); %初期時間の値を除く
            accel = accel(2:end, :, :, :); %初期時間の値を除く
            
            dt = obj.dt;
            
            Fml=0;
            %For low velocity range
            b0 = 0.1569;
            b1 = 2.450E-2;
            b2 = -7.415E-4;
            b3 = 5.975E-5;
            
            c0 = 0.07224;
            c1 = 9.681E-2;
            c2 = 1.075E-3;
            
            Fcruise = b0 + velocity*b1 + velocity.^2*b2 + velocity.^3*b3;
            Facc = accel.*(c0 + velocity*c1 + velocity.^2*c2).*(accel>0);
            Fml = (Fcruise + Facc)*dt;
        end
        
        %一台あたりの燃料消費量の計算
        function f = calculate_fuel_consumption(obj, time_series)
            dt = obj.dt;

            position = time_series(:, :, 1, :);
            velocity = time_series(:, :, 2, :);
            accel = time_series(:, :, 3, :);
            fuel_consumption = obj.FuelConsumptionGF(position, velocity, accel);
            fuel_consumption = fuel_consumption(~isnan(fuel_consumption));
            
            ml_to_l = 1/1000; %単位をmlからlに変換する
            m_to_km = 1/1000; %単位をmからkmに変換する
            vehicle_count = size(time_series, 2);
            drive_distance = m_to_km * (obj.road_length + 260);
            r = size(time_series, 4);
 
            f = ml_to_l * sum(fuel_consumption, 'all')/vehicle_count/drive_distance/r;
        end
        
        %平均速度の計算
        function a = calculate_average_velocity(obj, time_series)
            dt = obj.dt;
            
            %平均速度の計算(時速)
            velocity = time_series(:, :, 2, :);
            velocity = velocity(~isnan(velocity));
            average_velocity = mean(velocity);
            
            mps_to_kmph = 3.6;
            a = mps_to_kmph * average_velocity;% 単位をm/sからkm/hに変換する
        end
        
        %時系列データを車ごとにまとめる
        %いま、データは車1の0秒後1秒後2秒後...車2の0秒後1秒後2秒後...車3の..というふうに1秒ずつのデータとして入っている

        %車の移動時間の平均を求める
        function a = calculate_ave_travel_time(obj, time_series)
            position = time_series(:, :, 1, :);
            travel_time = sum(~isnan(position), 1) * obj.dt;
            
            %平均をここで求める
            %2行r列の結果を出す
            average = mean(travel_time, 2);
            
            %r回のデータの平均の平均を求める
            average = mean(average, 4);
            
            a = average;
        end
        
        %車の移動時間の標準偏差を求める
        function s = calculate_std_travel_time(obj, time_series)
            position = time_series(:, :, 1, :);
            travel_time = sum(~isnan(position), 1) * obj.dt;
            
            standard_deviation = std(travel_time, 1, 2);
            standard_deviation = mean(standard_deviation, 4);
            
            s = standard_deviation;
        end
        
        %車の待ち時間の平均を求める
        function a = calculate_ave_waiting_time(obj, time_series)
            velocity = time_series(:, :, 2, :);
            %速度が要素をNaNに置き換える
            invalid_index = (isnan(velocity) | velocity >= obj.delta_v_threshold);
            velocity(invalid_index) = NaN;
            waiting_time = sum(~isnan(velocity), 1) * obj.dt;
            
            %平均はここで求める
            average = mean(waiting_time, 2);
            %r回のデータの平均の平均を求める
            average = mean(average, 4);
            
            a = average;
        end
        
        %車の待ち時間の標準偏差を求める
        function s = calculate_std_waiting_time(obj, time_series)
            velocity = time_series(:, :, 2, :);
            %速度が要素をNaNに置き換える
            invalid_index = (isnan(velocity) | velocity >= obj.delta_v_threshold);
            velocity(invalid_index) = NaN;
            waiting_time = sum(~isnan(velocity), 1) * obj.dt;
            
            %標準偏差はここで求める
            standard_deviation = std(waiting_time, 1, 2);
            %r回のデータの標準偏差の平均を求める
            standard_deviation = mean(standard_deviation, 4);
            
            s = standard_deviation;
        end
        
        function table = generate_xlsm_data(obj, time_series, R)
            %データを出力
            id_number = size(time_series, 2);
            time = obj.dt * repmat([0:size(time_series, 1)-1]', id_number, 1);
            id = repelem(1:id_number, size(time_series, 1));
            id = reshape(id, [], 1);
            position = reshape(time_series(:, :, 1, R), [], 1);
            mps_to_kmph = 3.6;
            velocity = mps_to_kmph * reshape(time_series(:, :, 2, R), [], 1);
            accel = reshape(time_series(:, :, 3, R), [], 1);
            lane = reshape(time_series(:, :, 4, R), [], 1);
            
            table = [id time position velocity accel lane];
        end
    end
end
%シミュレーション結果をグラフにする関数まとめ
classdef Graphics
    properties
        dt
        road_length
    end
    
    methods
        %コンストラクタ
        function obj = Graphics(init_value)
            obj.dt = init_value.dt;
            obj.road_length = init_value.road_length;
        end
        
        %位置をグラフにする
        function graph_position(obj, time_series, t, r)
            lane_index = time_series(:, :, 4, r);
            right_vehicle_position = time_series(:, :, 1, r);
            right_vehicle_position(lane_index == 1) = NaN;
            left_vehicle_position = time_series(:, :, 1, r);
            left_vehicle_position(lane_index == 0) = NaN;
            
            figure;hold on;
            plot(t, time_series(:, :, 1, r), '-r')
            plot(t, right_vehicle_position, '-r')
            plot(t, left_vehicle_position, '-b')
        end
        
        %速度をグラフにする
        function graph_velocity(obj, time_series, t, r)
            lane_index = time_series(:, :, 4, r);
            left_vehicle_velocity = time_series(:, :, 2, r);
            left_vehicle_velocity(lane_index == 1) = NaN;
            right_vehicle_velocity = time_series(:, :, 2, r);
            right_vehicle_velocity(lane_index == 0) = NaN;
            
            figure;hold on;
            plot(t, time_series(:, :, 2, r), '-r')
            plot(t, left_vehicle_velocity, '-r')
            plot(t, right_vehicle_velocity, '-b')
        end
        
        %加速度をグラフにする
        function graph_accel(obj, time_series, t, r)
            lane_index = time_series(:, :, 4, r);
            left_vehicle_accel = time_series(:, :, 3, r);
            left_vehicle_accel(lane_index == 1) = NaN;
            right_vehicle_accel = time_series(:, :, 3, r);
            right_vehicle_accel(lane_index == 0) = NaN;
            
            figure;hold on;
            plot(t, time_series(:, :, 3, r), '-r')
            plot(t, left_vehicle_accel, '-r')
            plot(t, right_vehicle_accel, '-b')
        end
        
        %道路外の車の情報を除く
        function [previous, proposed] = remove_data_out_of_range(obj, previous_time_series, proposed_time_series)
            previous_position = previous_time_series(:, :, 1, :);
            previous_velocity = previous_time_series(:, :, 2, :);
            previous_accel = previous_time_series(:, :, 3, :);
            valid_index = (previous_position <= -130 | obj.road_length + 130 <= previous_position);
            previous_position(valid_index) = NaN;
            previous_velocity(valid_index) = NaN;
            previous_accel(valid_index) = NaN;
            previous_time_series(:, :, 1, :) = previous_position;
            previous_time_series(:, :, 2, :) = previous_velocity;
            previous_time_series(:, :, 3, :) = previous_accel;
            
            proposed_position = proposed_time_series(:, :, 1, :);
            proposed_velocity = proposed_time_series(:, :, 2, :);
            proposed_accel = proposed_time_series(:, :, 3, :);
            valid_index = (proposed_position <= -130 | obj.road_length + 130 <= proposed_position);
            proposed_position(valid_index) = NaN;
            proposed_velocity(valid_index) = NaN;
            proposed_accel(valid_index) = NaN;
            proposed_time_series(:, :, 1, :) = proposed_position;
            proposed_time_series(:, :, 2, :) = proposed_velocity;
            proposed_time_series(:, :, 3, :) = proposed_accel;
            
            previous = previous_time_series;
            proposed = proposed_time_series;
        end
        
        %単位時間あたりの燃料消費量の計算
        function Fml = FuelConsumptionGF(obj, velocity, accel)
            dt = obj.dt;
            
            Fml=0;
            %For  low velocity range
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
        
        %総燃料消費量の計算
        function [previous, proposed] = calculate_fuel_consumption(obj, previous_time_series, proposed_time_series)
            dt = obj.dt;
            mps_to_kmph = 3600/1000;
            %従来の手法
            AFC = obj.FuelConsumptionGF(previous_time_series(:, :, 2, :), previous_time_series(:, :, 3, :));
            AFC = AFC(~isnan(AFC));
            %提案された手法
            FC = obj.FuelConsumptionGF(proposed_time_series(:, :, 2, :), proposed_time_series(:, :, 3, :));
            FC = FC(~isnan(FC));
            
            ml_to_l = 1/1000;
            previous = ml_to_l * sum(AFC, 'all');% 単位をmlからlに変換する
            proposed = ml_to_l * sum(FC, 'all');% 単位をmlからlに変換する
        end
        
        %平均速度の計算
        function [previous, proposed] = calculate_average_velocity(obj, previous_time_series, proposed_time_series)
            dt = obj.dt;
            
            %平均速度の計算(時速)
            %従来の手法
            previous_velocity = previous_time_series(:, :, 2, :);
            previous_velocity = previous_velocity(~isnan(previous_velocity));
            previous_average_velocity = mean(previous_velocity);
            %提案された手法
            proposed_velocity = proposed_time_series(:, :, 2, :);
            proposed_velocity = proposed_velocity(~isnan(proposed_velocity));
            proposed_average_velocity = mean(proposed_velocity);
            
            mps_to_kmph = 3600/1000;
            previous = mps_to_kmph * previous_average_velocity;% 単位をm/sからkm/hに変換する
            proposed = mps_to_kmph * proposed_average_velocity;% 単位をm/sからkm/hに変換する
        end
    end
end
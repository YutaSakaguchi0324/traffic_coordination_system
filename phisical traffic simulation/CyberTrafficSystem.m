classdef CyberTrafficSystem
    properties
        %設定
        traffic_settings
        
        %シミュレーションに関する変数
        dt %シミュレーションの単位時間
        start_line %この位置からシミュレーションを始める
        end_line %これを過ぎるとシミュレーションから除外される

        %車に関する変数
        vehicle_length %車の長さ
        gap_min %車の最小車間距離
        Vmax %最大速度
        Vdes %理想速度
        Vmin %最小速度
        Amax %最大加速度
        Ades %理想減速度
        Amin %最小加速度
        
        %最適化に関する変数
        time_horizon
        group_vehicle_count
        
        %交通環境
        road_start %2車線道路の始まり(合流地点)
        road_length %2車線道路の長さ(分岐地点)
    end
    
    methods
        %コンストラクタ
        function obj = CyberTrafficSystem(settings)
            obj.traffic_settings = settings;
            obj.dt = settings.dt;
            
            obj.start_line = settings.road_start - 200;
            obj.end_line = settings.road_length + 200;
            
            obj.vehicle_length = settings.vehicle_length;
            obj.gap_min = settings.gap_min;
            obj.Vmax = settings.Vmax;
            obj.Vdes = settings.Vdes;
            obj.Vmin = settings.Vmin;
            obj.Amax = settings.Amax;
            obj.Ades = settings.Ades;
            obj.Amin = settings.Amin;
            
            obj.time_horizon = settings.time_horizon;
            obj.group_vehicle_count = settings.group_vehicle_count;
            
            obj.road_start = settings.road_start;
            obj.road_length = settings.road_length;
        end

        %車追従モデル関数(推定値)
        %PhysicalTrafficSimulationクラスの同名メソッドのコピー
        %実空間の車線変更モデルの推定値
        function r = car_following_model(obj, x_leader, x_control, v_leader, v_control)
            %入力引数は配列でも可
            if isempty(x_control) && isempty(v_control)
                next_velocity = [];
                
            elseif ~isempty(x_control) && ~isempty(v_control)
                if isempty(x_leader) && isempty(v_leader)
                    x_leader = 1000 + x_control;
                    v_leader = v_control;
                    
                elseif ~isempty(x_leader) && ~isempty(v_leader)
                    
                else
                    error('xlとvlは「両値とも実数」か「両値ともNaN」のいずれかでなければいけません')
                end
                
                if x_control <= obj.start_line
                    v_des = 16;
                else
                    v_des = obj.Vdes;
                end
                
                %IDMモデル
                dt = obj.dt;
                gap_min = obj.gap_min;
                gap = x_leader - x_control - obj.vehicle_length;
                a = obj.Amax;
                b = -obj.Ades;
                time_headway = 1.5;
                delta = 4;
               
                s = gap_min + max(0, v_control*time_headway + 0.5*v_control.*(v_control-v_leader)/sqrt(a*b));
                next_velocity = v_control + a*(1 - (v_control/v_des).^delta - (s./gap).^2)*dt;
                
            else
                error('xfとvfは「両値とも実数」か「両値ともNaN」のいずれかでなければいけません')
            end
            
            r = next_velocity;
        end
        
        %車を追従させる(1レーン)
        %車がすべて同じレーンにいるとき
        function r = follow_cars_on_one_lane(obj, leader_state, state, other_state)
            dt = obj.dt;
            Vmin = obj.Vmin;
            Amin = obj.Amin;
            road_length = obj.road_length;
            
            %停止車両閾値
            delta_v_threshold = 2.6;
            
            for i = 1:size(state, 2)
                if i == 1
                    if size(leader_state, 2) == 0
                        xl = [];
                        vl = [];
                    else
                        xl = leader_state(2);
                        vl = leader_state(3);
                    end
                    xc = state(2, i);
                    vc = state(3, i);
                else
                    xl = state(2, i-1);
                    xc = state(2, i);
                    vl = state(3, i-1);
                    vc = state(3, i);
                end

                next_velocity1 = max([0, vc + Amin*dt, obj.car_following_model(xl, xc, vl, vc)]);
                
                %目的地と異なる車線にいかないようにする
                if state(6, i) == 1
                    if state(5, 1) == 0
                        play = 0;
                    else
                        play = 15;
                    end
                    next_velocity2 = obj.car_following_model(road_length - play, xc, 0,  vc);
                else
                    next_velocity2 = next_velocity1;
                end
                
                %停止した車に道を譲る
                stop_vehicle = other_state(:, other_state(3, :) < delta_v_threshold);
                if size(stop_vehicle, 2) == 0
                    next_velocity3 = obj.car_following_model([], xc, [], vc);
                else
                    stop_velocity = obj.car_following_model(stop_vehicle(2, 1), xc, 0, vc);
                    if stop_velocity - vc >= Amin*dt && stop_velocity >= Vmin
                        next_velocity3 = stop_velocity;
                    else
                        next_velocity3 = obj.car_following_model([], xc, [], vc);
                    end
                end
                
                next_velocity = min([next_velocity1 next_velocity2 next_velocity3]);
                state(4, i) = (next_velocity - state(3, i))/dt;
                state(3, i) = next_velocity;
                state(2, i) = state(2, i) + next_velocity*dt;
            end
            
            r = state;
        end
        
        %車を追従させる(2レーン)
        %車が2車線道路のどちらかのレーンにいるとき
        function r = follow_vehicles(obj, leader_state, state)
            %車を追従させる
            right_state = state(:, (state(5, :) == 0));
            left_state = state(:, (state(5, :) == 1));
            
            if size(leader_state, 2) == 0
                right_leader_state = [];
                left_leader_state = [];
            else
                right_leader_state = leader_state(:, leader_state(5, :) == 0);
                left_leader_state = leader_state(:, leader_state(5, :) == 1);
            end
            
            right_state = obj.follow_cars_on_one_lane(right_leader_state, right_state, left_state);
            left_state = obj.follow_cars_on_one_lane(left_leader_state, left_state, right_state);
            next_state = state;
            next_state(:, state(5, :) == 0) = right_state;
            next_state(:, state(5, :) == 1) = left_state;
            
            r = next_state;
        end
        
        %A,B,Cグループに分ける
        %A,Cグループは最適化を行わない、Bグループのみ最適化を行う
        function [r1, r2, r3] = split_into_ABC(obj, state)
            N = obj.group_vehicle_count;

            state = reshape(state, 7, N, []);
            A_state = state(:, :, state(2, 1, :) >= obj.road_length - 200);
            B_state = state(:, :, state(2, 1, :) >= -200 & state(2, 1, :) < obj.road_length - 200);
            C_state = state(:, :, state(2, 1, :) < -200);
            
            A_state = reshape(A_state, 7, [], 1);
            B_state = reshape(B_state, 7, [], 1);
            C_state = reshape(C_state, 7, [], 1);
            
            r1 = A_state;
            r2 = B_state;
            r3 = C_state;
        end
        
        %AグループとCグループの車を追従させる
        function [r1, r2] = follow_vehicles_T_seconds(obj, leader_state_T, state)
            %stateは二次元配列
            %leader_state_Tは三次元配列
            %leader_state_Tは常に二台
            T = obj.time_horizon;
            if size(state, 2) > 0
                state_T = zeros(7, size(state, 2), T+1);
                state_T = state_T + state;
                for t = 1:T
                    %車を追従させる
                    state_T(:, :, t+1) = obj.follow_vehicles(leader_state_T, state_T(:, :, t));
                end
                
                right_state = state(:, state(5, :) == 0);
                left_state = state(:, state(5, :) == 1);
                %例外処理
                if size(right_state, 2) > 0
                    right_tail_index = find(state(1, :) == right_state(1, end));
                    right_tail_state_T = state_T(:, right_tail_index, :);
                else
                    right_tail_state_T = state_T(:, state_T(5, :, 1) == 2, :);
                end
                
                if size(left_state, 2) > 0
                    left_tail_index = find(state(1, :) == left_state(1, end));
                    left_tail_state_T = state_T(:, left_tail_index, :);
                else
                    left_tail_state_T = state_T(:, state_T(5, :, 1) == 2, :);
                end
                
                r1 = state_T;
                r2 = [right_tail_state_T left_tail_state_T];

            else
                
                r1 = [];
                r2 = [];
            end
        end
        
        %Bグループを最適化する
        function [r1, r2] = optimize_vehicles(obj, leader_state_T, B_state)
            %leader_state_Tは常に二台
            T = obj.time_horizon;
            N = obj.group_vehicle_count;
            %Bグループの処理
            if size(B_state, 2) > 0
                %stateはBグループ全体の車の状態
                %stateに対応する、T+1秒間の情報を含むstate_Tをつくる
                %state_Tは最終的に第一出力として返される
                B_state_T = zeros(7, size(B_state, 2), T+1); 
                B_state_T = B_state_T + B_state;
                for group = 1:size(B_state, 2)/N
                    group_state = B_state(:, N*(group-1)+1:N*group);
                    
                    %最適化を実行する
                    PO = PartialOptimize(leader_state_T, group_state, obj.traffic_settings);
                    velocity = PO.partial_optimize();
                    position = PO.calculate_positions(velocity);
                    velocity = reshape(velocity, T+1, []);
                    position = reshape(position, T+1, []);
                    
                    group_state_T = zeros(7, N, T+1);
                    group_state_T = group_state_T + group_state;
                    group_state_T = permute(group_state_T, [3 2 1]);
                    group_state_T(:, :, 2) = position;
                    group_state_T(:, :, 3) = velocity;
                    group_state_T(2:T+1, :, 4) = (velocity(2:T+1, :) - velocity(1:T, :))*obj.dt;
                    group_state_T = permute(group_state_T, [3 2 1]);
                    
                    B_state_T(:, N*(group-1)+1:N*group, :) = group_state_T;
                    
                    %各々の車線の最後尾車両を記憶する
                    if size(leader_state_T, 2) > 0
                        right_leader_state_T = leader_state_T(:, leader_state_T(5, :, 1) == 0, :);
                    else
                        right_leader_state_T = leader_state_T;
                    end
                    right_group_state_T = group_state_T(:, group_state(5, :) == 0, :);
                    right_state_T = [right_leader_state_T right_group_state_T];
                    if size(right_state_T, 2) > 0
                        right_state_T = right_state_T(:, end, :);
                    end
                    
                    if size(leader_state_T, 2) > 0
                        left_leader_state_T = leader_state_T(:, leader_state_T(5, :, 1) == 1, :);
                    else
                        left_leader_state_T = [];
                    end
                    left_group_state_T = group_state_T(:, group_state(5, :) == 1, :);
                    left_state_T = [left_leader_state_T left_group_state_T];
                    if size(left_state_T, 2) > 0
                        left_state_T = left_state_T(:, end, :);
                    end
                    
                    leader_state_T = [right_state_T left_state_T];
                end
                
                right_state = B_state(:, B_state(5, :) == 0);
                left_state = B_state(:, B_state(5, :) == 1);
                %例外処理
                if size(right_state, 2) > 0
                    right_tail_index = find(B_state(1, :) == right_state(1, end));
                    right_tail_state_T = B_state_T(:, right_tail_index, :);
                else
                    right_tail_state_T = B_state_T(:, B_state_T(5, :, 1) == 2, :);
                end
                
                if size(left_state, 2) > 0
                    left_tail_index = find(B_state(1, :) == left_state(1, end));
                    left_tail_state_T = B_state_T(:, left_tail_index, :);
                else
                    left_tail_state_T = B_state_T(:, B_state_T(5, :, 1) == 2, :);
                end
                
                r1 = B_state_T;
                r2 = [right_tail_state_T left_tail_state_T];
            else
                r1 = [];
                r2 = [];
            end
        end

    end
    
end
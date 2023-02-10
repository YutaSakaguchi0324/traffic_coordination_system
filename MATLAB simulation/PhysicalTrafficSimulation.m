classdef PhysicalTrafficSimulation
    properties
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
        
        %挿入する車に関する変数
        inserted_vehicle_count %一度に挿入される車の台数
        insert_times %挿入する回数
        insert_time_interval %挿入されるまでの間隔
        sp
        er
        
        %交通環境
        road_start %2車線道路の始まり(合流地点)
        road_length %2車線道路の長さ(分岐地点)
        
        %その他
        delta_v_threshold %車が停止したとみなす速度
        
    end
    
    methods
        %コンストラクタ
        function obj = PhysicalTrafficSimulation(settings)
            %シミュレーションに関する変数
            obj.dt = settings.dt;
            obj.start_line = settings.road_start - 200;
            obj.end_line = settings.road_length + 200;
            
            %車に関する変数
            obj.vehicle_length = settings.vehicle_length;
            obj.gap_min = settings.gap_min;
            obj.Vmax = settings.Vmax;
            obj.Vdes = settings.Vdes;
            obj.Vmin = settings.Vmin;
            obj.Amax = settings.Amax;
            obj.Ades = settings.Ades;
            obj.Amin = settings.Amin;
            
            %挿入する車列に関する変数
            obj.inserted_vehicle_count = settings.inserted_vehicle_count;
            obj.insert_times = settings.insert_times;
            obj.insert_time_interval = settings.insert_time_interval;
            obj.sp = settings.sp;
            obj.er = settings.er;
            
            %交通環境変数
            obj.road_start = settings.road_start;
            obj.road_length = settings.road_length;
            
            %その他
            obj.delta_v_threshold = settings.delta_v_threshold;
        end
        
        %ランダム配列の作成
        function r = generate_random_array(obj, repetitions)
            %ランダム四次元配列を生成する
            N = obj.inserted_vehicle_count;
            T = obj.insert_times;
            
            random_array = zeros(N, T, 7, repetitions);
            %車のid
            random_array(:, :, 1, :) = random_array(:, :, 1, :) + reshape([1:N*T], N, T);
            %車の位置
            if rem(N, 2) == 0
                n = N;
            else
                n = N+1;
            end
            array = zeros(2, n);
            array(2, :) = obj.er;
            array = array + obj.sp*[0:n-1];
            
            random_position = -3 + 6*rand(N, T, repetitions);
            random_array(:, :, 2, :) = random_position - 200 + array(1:N)';
            %車の速度
            random_array(:, :, 3, :) = 16;
            
            %車の加速度
            random_array(:, :, 4, :) = NaN;
            
            %車の車線の番号
            random_array(rem(1:N, 2) == 1, :, 5, :) = 0;
            random_array(rem(1:N, 2) == 0, :, 5, :) = 1;
            
            %車が車線変更するかどうか
            random_lanechange = randi([1 10], N, T, repetitions);
            random_lanechange(random_lanechange <= 3) = 1;
            random_lanechange(random_lanechange > 3) = 0;
            random_array(:, :, 6, :) = random_lanechange;
            
            %車線変更クールダウン
            random_array(:, :, 7, :) = 0;
            
            %値を返す
            r = random_array;
        end
        
        %車列を挿入する
        %stateの再構成
        function [r1, r2] = insert_vehicles(obj, insert_information)
            t = insert_information.t;
            r = insert_information.r;
            count = insert_information.count;
            random_array = insert_information.random_array;
            state_variable = insert_information.state_variable;
            
            if rem(t, obj.insert_time_interval) == 0 && count <= obj.insert_times
                addition = random_array(:, count, :, r);
                addition = reshape(addition, obj.inserted_vehicle_count, []);
                addition = addition';
                if size(state_variable, 2) > 0
                    if state_variable(2, end-1) <= -200 - obj.sp
                        distance = state_variable(2, end-1) + 200 + obj.sp;
                        addition(2, :) = addition(2, :) + distance - 5;
                    end
                end
                state_variable = [state_variable addition];
                
                count = count + 1;
            end
            
            r1 = count;
            r2 = state_variable;
        end
        
        % 車追従モデル関数
        % 純粋関数なのでこのままで良い
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
            
            for i = 1:size(state, 2)
                %前方の車とぶつからないようにする
                if i == 1
                    if size(leader_state, 2) == 0
                        xl = [];
                        vl = [];
                    else
                        xl = leader_state(2);
                        vl = leader_state(3);
                    end
                else
                    xl = state(2, i-1);
                    vl = state(3, i-1);
                end
                
                xc = state(2, i);
                vc = state(3, i);

                next_velocity1 = max([0, vc + Amin*dt, obj.car_following_model(xl, xc, vl, vc)]);
                
                %目的地と異なる車線にいかないようにする
                if state(6, i) == 1
                    if state(5, 1) == 0
                        play = 0;
                    elseif state(5, 1) == 1
                        play = 15;
                    end
                    next_velocity2 = obj.car_following_model(road_length - play, xc, 0,  vc);
                else
                    next_velocity2 = next_velocity1;
                end
                
                %停止した車に道を譲る
                stop_vehicle = other_state(:, other_state(3, :) < obj.delta_v_threshold);
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
        
        % 車線変更ができるか判定する
        % 純粋関数なのでこのままで良い
        function r = lane_change_possible(obj, surroundings)
            %インスタンス変数
            dt = obj.dt;     
            vehicle_length = obj.vehicle_length;
            gap_min = obj.gap_min;
            Vmin = obj.Vmin;
            Ades = obj.Ades;
            Amin = obj.Amin;
            
            %入力引数
            xl_another = surroundings.xl_another;
            x_control = surroundings.x_control;
            xf_another = surroundings.xf_another;
            vl_another = surroundings.vl_another;
            v_control = surroundings.v_control;
            vf_another = surroundings.vf_another;
            
            %先行車と対象車との車線変更可能性
            if xl_another - x_control < vehicle_length + gap_min
                A = 0;
            elseif isempty(xl_another) && isempty(vl_another)
                A = 1;
            elseif ~isempty(xl_another) && ~isempty(vl_another)
                velocity = obj.car_following_model(xl_another, x_control, vl_another, v_control);
                A = (velocity - v_control >= Ades/dt && velocity >= Vmin);
            else
                error('xl_another vl_another')
            end
            
            %対象車と後続車の車線変更可能性
            if x_control - xf_another < vehicle_length + gap_min
                B = 0;
            elseif isempty(xf_another) && isempty(vf_another)
                B = 1;
            elseif ~isempty(xf_another) && ~isempty(vf_another)
                velocity = obj.car_following_model(x_control, xf_another, v_control, vf_another);
                B =(velocity - vf_another >= Ades/dt && velocity >= Vmin);
            else
                error('xf_another vf_another')
            end
            
            if  A && B
                bool = true;
            else
                bool = false;
            end
            
            r = bool;
        end
        
        % 車線変更するべきか決める
        % 純粋関数なのでこのままで良い
        function r = lane_change_model(obj, surroundings)
            %車線変更モデルMOBILをつかう
            %入力引数
            xl_another = surroundings.xl_another;
            xl_current = surroundings.xl_current;
            x_control = surroundings.x_control;
            vl_another = surroundings.vl_another;
            vl_current = surroundings.vl_current;
            v_control = surroundings.v_control;
            
            %対象の車の、車線変更前後の加速度を計算する
            ac = obj.car_following_model(xl_current, x_control, vl_current, v_control) - v_control;
            tilde_ac = obj.car_following_model(xl_another, x_control, vl_another, v_control) - v_control;
            
            %加速度の閾値を決定する
            %閾値は0から負のある値に変動していく
            delta_a_threshold = -5/obj.road_length * x_control;
            
            %車線変更するべきか判定する
            %車線変更できるか判定する
            if obj.lane_change_possible(surroundings)
                if tilde_ac - ac > delta_a_threshold/obj.dt
                    bool = true;
                else
                    bool = false;
                end
            else
                bool = false;
            end
            
            r = bool;
        end
        
        %車を車線変更させる
        function r = change_lanes(obj, state)
            %車線変更できる車は車線変更する
            %車線変更クールダウン
            cooldown_bool = (state(7, :) > 0);
            state(7, cooldown_bool) = state(7, cooldown_bool)  - 1;
            
            vehicle_count = size(state, 2);
            
            for i = 1:vehicle_count
                
                if state(6, i) == 1 && state(2, i) >= obj.road_start
                    current_leaders = state(:, (state(5, :) == state(5, i) & state(2, :) > state(2, i)));
                    another_leaders = state(:, (state(5, :) ~= state(5, i) & state(2, :) > state(2, i)));
                    current_followers = state(:, (state(5, :) == state(5, i) & state(2, :) < state(2, i)));
                    another_followers = state(:, (state(5, :) ~= state(5, i) & state(2, :) < state(2, i)));
                    
                    if isempty(current_leaders)
                        xl_current = [];
                        vl_current = [];
                        l_current_cooldown = 0;
                    else
                        xl_current = current_leaders(2, end);
                        vl_current = current_leaders(3, end);
                        l_current_cooldown = current_leaders(7, end);
                    end
                    
                    if isempty(another_leaders)
                        xl_another = [];
                        vl_another = [];
                        l_another_cooldown = 0;
                    else
                        xl_another = another_leaders(2, end);
                        vl_another = another_leaders(3, end);
                        l_another_cooldown = another_leaders(7, end);
                    end
                    
                    if isempty(current_followers)
                        f_current_cooldown = 0;
                    else
                        f_current_cooldown = current_followers(7, 1);
                    end
                    
                    if isempty(another_followers)
                        xf_another = [];
                        vf_another = [];
                        f_another_cooldown = 0;
                    else
                        xf_another = another_followers(2, 1);
                        vf_another = another_followers(3, 1);
                        f_another_cooldown = another_followers(7, 1);
                    end
                    
                    x_control = state(2, i);
                    v_control = state(3, i);
                    
                    if l_current_cooldown == 0 && l_another_cooldown == 0 && f_current_cooldown == 0 && f_another_cooldown == 0
                        
                        surroundings.xl_current = xl_current;
                        surroundings.xl_another = xl_another;
                        surroundings.x_control = x_control;
                        surroundings.xf_another = xf_another;
                        surroundings.vl_current = vl_current;
                        surroundings.vl_another = vl_another;
                        surroundings.v_control = v_control;
                        surroundings.vf_another = vf_another;
                        if obj.lane_change_model(surroundings)
                            
                            state(5, i) = ~state(5, i);
                            state(6, i) = 0;
                            
                            if v_control > 1
                                state(7, i) = 0;
                            else
                                state(7, i) = 5;
                            end
                            
                        end
                    end
                end
                
            end
            
            r = state;
        end

        %車をシミュレーションから削除
        function r = delete_vehicles(obj, state)
            %道路から出ていった車を排除する
            N = obj.inserted_vehicle_count;
            if state(2, N) >= obj.end_line && all(state(6, 1:N) == 0)
                state(:, 1:N) = [];
            end
            r = state;
        end
        
    end
    
    methods (Static)
        %車を前から順に整列させる
        function r = sort_cars(state_variable)
            %車を前の位置から並べ直す
            [B, index] = sort(state_variable(2, :), 'descend');
            r = state_variable(:, index);
        end
    end
end
%最適化するクラス
classdef PartialOptimize
    properties
        %設定
        traffic_settings
        
        %グループに関する変数
        right_leader_state_T
        left_leader_state_T
        group_state
        right_group_state
        left_group_state
        group_vehicle_count
        right_vehicle_count
        left_vehicle_count
        
        %シミュレーションに関する変数
        dt
        
        %車の設定
        Vmax
        Vdes
        Vmin
        Amax
        Ades
        Amin
        
        %最適化に関する変数
        time_horizon
        alpha
        w1
        w2
        w3
        
        %その他の変数
        nearly_lower_triangular_matrix % 速度から位置に変換する行列
        risk_matrix % 車線変更する車以外は評価をゼロにする行列
        differential_acceleration_matrix % 加速度微分に使う行列
    end
    
    methods
        %コンストラクタ
        function obj = PartialOptimize(leader_state_T, group_state, settings)
            T = settings.time_horizon;
            N = settings.group_vehicle_count;
            leader_state_T = reshape(leader_state_T, 7, [], T+1);
            %設定
            obj.traffic_settings = settings;
            
            %車の状態変数
            obj.right_leader_state_T = leader_state_T(:, leader_state_T(5, :, 1) == 0, :);
            obj.left_leader_state_T = leader_state_T(:, leader_state_T(5, :, 1) == 1, :);
            obj.group_state = group_state;%分割されたグループのうちのひとつ
            obj.right_group_state = group_state(:, group_state(5, :) == 0);
            obj.left_group_state = group_state(:, group_state(5, :) == 1);
            obj.right_vehicle_count = size(obj.right_group_state, 2);
            obj.left_vehicle_count = size(obj.left_group_state, 2);
            obj.group_vehicle_count = settings.group_vehicle_count;%グループの中の車の数
            %シミュレーションの変数
            obj.dt = settings.dt;%シミュレーションの単位時間
            
            %車の設定
            obj.Vmax = settings.Vmax;%最大速度
            obj.Vdes = settings.Vdes;%理想速度
            obj.Vmin = settings.Vmin;%最小速度
            obj.Amax = settings.Amax;%最大加速度
            obj.Ades = settings.Ades;%理想加速度
            obj.Amin = settings.Amin;%最小加速度
            
            %最適化に関する変数
            obj.time_horizon = T;%予測ホライゾン
            obj.alpha = settings.alpha;
            obj.w1 = settings.w1;%目的関数の速度係数
            obj.w2 = settings.w2;%目的関数の加速度係数
            obj.w3 = settings.w3;%目的関数の車線変更リスク係数
            
            %その他の変数
            %速度から位置に変換する行列
            matrix = obj.dt * ones(T+1);
            matrix = tril(matrix);
            matrix(:, 1) = 0;
            obj.nearly_lower_triangular_matrix = matrix;%疑似下三角行列
            
            %車線変更する車以外は評価をゼロにする行列
            right_leader_lanechange = obj.right_leader_state_T(6, :, 1);
            left_leader_lanechange = obj.left_leader_state_T(6, :, 1);
            right_group_lanechange = obj.right_group_state(6, :);
            left_group_lanechange = obj.left_group_state(6, :);
            right_lanechange = [right_group_lanechange right_leader_lanechange];
            left_lanechange = [left_group_lanechange left_leader_lanechange];
            risk_matrix = reshape(right_lanechange, [], 1) | reshape(left_lanechange, 1, []);
            risk_matrix = zeros(size(risk_matrix, 1), size(risk_matrix, 2), T) + risk_matrix;
            if size(right_lanechange, 2) > 0 && size(left_lanechange, 2) > 0
                risk_matrix(end, end, :) = 0;
            end
            obj.risk_matrix = risk_matrix;
            
            %加速度微分を求める行列
            matrix = eye(T+1);
            matrix(2:T+1, 1:T) = matrix(2:T+1, 1:T) - eye(T);
            matrix(2:T, 2:T) = matrix(2:T, 2:T) + eye(T-1);
            matrix(1:T, 2:T+1) = matrix(1:T, 2:T+1) - eye(T);
            obj.differential_acceleration_matrix = kron(eye(N), matrix);
            
        end
        
        %速度配列から位置配列を計算する関数
        function r = calculate_positions(obj, velocity)
            %入力引数は一次元配列とする
            T = obj.time_horizon;
            velocity  = reshape(velocity, T+1, []);
            position = obj.nearly_lower_triangular_matrix * velocity + obj.group_state(2, :);
            
            %出力も一次元配列
            r = reshape(position, [], 1);%縦ベクトル
        end
        
        %初期値x0を生成する
        function r = generate_x0(obj)
            right_leader_state_T = obj.right_leader_state_T;
            left_leader_state_T = obj.left_leader_state_T;
            group_state = obj.group_state;
            right_group_state = obj.right_group_state;
            left_group_state = obj.left_group_state;
            dt = obj.dt;
            T = obj.time_horizon;
            N = obj.group_vehicle_count;
            CTS = CyberTrafficSystem(obj.traffic_settings);
            
            %右車線の車
            R = obj.right_vehicle_count;
            
            right_vehicle_velocity = zeros(T+1, R);
            right_vehicle_position = zeros(T+1, R);
            right_vehicle_velocity(1, :) = right_group_state(3, :);
            right_vehicle_position(1, :) = right_group_state(2, :);
            for n = 1:R
                for t = 1:T
                    if n == 1
                        xl = right_leader_state_T(2, :, t);
                        vl = right_leader_state_T(3, :, t);
                    else
                        xl = right_vehicle_position(t, n-1);
                        vl = right_vehicle_velocity(t, n-1);
                    end
                    xc = right_vehicle_position(t, n);
                    vc = right_vehicle_velocity(t, n);

                    next_velocity = CTS.car_following_model(xl, xc, vl, vc);
                    right_vehicle_velocity(t+1, n) = next_velocity;
                    right_vehicle_position(t+1, n) = right_vehicle_position(t, n) + next_velocity*dt;
                end
            end
            
            %左車線の車
            L = obj.left_vehicle_count;
            left_vehicle_velocity = zeros(T+1, L);
            left_vehicle_position = zeros(T+1, L);
            left_vehicle_velocity(1, :) = left_group_state(3, :);
            left_vehicle_position(1, :) = left_group_state(2, :);
            for n = 1:L
                for t = 1:T
                    if n == 1
                        xl = left_leader_state_T(2, :, t);
                        vl = left_leader_state_T(3, :, t);
                    else
                        xl = left_vehicle_position(t, n-1);
                        vl = left_vehicle_velocity(t, n-1);
                    end
                    xc = left_vehicle_position(t, n);
                    vc = left_vehicle_velocity(t, n);
                    
                    next_velocity = CTS.car_following_model(xl, xc, vl, vc);
                    left_vehicle_velocity(t+1, n) = next_velocity;
                    left_vehicle_position(t+1, n) = left_vehicle_position(t, n) + next_velocity*dt; 
                end
            end
            
            vehicle_velocity = zeros(T+1, N);
            vehicle_velocity(:, group_state(5, :) == 0) = right_vehicle_velocity;
            vehicle_velocity(:, group_state(5, :) == 1) = left_vehicle_velocity;
            r = reshape(vehicle_velocity, [], 1);
        end
            
        %目的関数を設定する
        function value = objective_function(obj, x)
            %目的関数
            right_leader_state_T = obj.right_leader_state_T;
            left_leader_state_T = obj.left_leader_state_T;
            group_state = obj.group_state;
            N = obj.group_vehicle_count;
            dt = obj.dt;
            T = obj.time_horizon;
            Vdes = obj.Vdes;
            alpha = obj.alpha;
            w1 = obj.w1;
            w2 = obj.w2;
            w3 = obj.w3;
            risk_matrix = obj.risk_matrix;

            %速度の評価
            %各車の初期値は評価しない
            velocity = reshape(x, T+1, []);
            velocity_evaluation = sum([(velocity(2:T+1, :) - Vdes).^2], 'all');
            
            %加速度の評価
            %各車の初期値は含まれない
            accel = x(2:T+1) - x(1:T);
            accel_evaluation = sum(accel.^2/dt^2);
            
            %リスク関数の評価
            %各車の初期値は評価しない
            position = obj.calculate_positions(x);
            position = reshape(position, T+1, []);
            
            right_leader_position = right_leader_state_T(2, :, 2:T+1);
            right_leader_position = permute(right_leader_position, [3 2 1]);
            right_position = position(2:T+1, group_state(5, :) == 0);
            right_position = [right_position right_leader_position];
            
            left_leader_position = left_leader_state_T(2, :, 2:T+1);
            left_leader_position = permute(left_leader_position, [3 2 1]);
            left_position = position(2:T+1, group_state(5, :) == 1);
            left_position = [left_position left_leader_position];
            %左車線と右車線の配列を作る
            distance = zeros(size(right_position, 2), size(left_position, 2), T);
            for t = 1:T
                distance(:, :, t) = right_position(t, :).' - left_position(t, :);
            end
            distance = risk_matrix .* distance;
            EXP = exp(-alpha*distance.^2);
            k = reshape(sum(position(2:T+1, :), 2), 1, 1, []);
            EXP = k .* risk_matrix .* EXP;
            
            risk_evaluation = sum(EXP, 'all');
            value = w1*velocity_evaluation + w2*accel_evaluation + w3*risk_evaluation;
            
            %{
            %目的関数の勾配
            %速度の評価
            velocity_gradient = 2*(x - Vdes);
            velocity_gradient(1:T+1:N*(T+1)) = 0;
            %加速度の評価
            accel_gradient = 2/dt^2 * obj.differential_acceleration_matrix * x;
            %車線変更リスクの評価
            distance_gradient = -2*alpha*distance .* EXP;
            R = obj.right_vehicle_count;
            risk_gradient = 0*x;
            for n = 1:N
                if n <= R
                    risk_gradient((n-1)*(T+1)+2:n*(T+1)) = sum(distance_gradient(n, :, :), 2);
                else
                    risk_gradient((n-1)*(T+1)+2:n*(T+1)) = sum(-distance_gradient(:, n-R, :), 1);
                end
            end
            gradient = w1*velocity_gradient + w2*accel_gradient + w3*risk_gradient;
            %}
        end
        
        %非線形制約関数
        function [C, Ceq] = car_following_constraints(obj, x)
            right_leader_state_T = obj.right_leader_state_T;
            left_leader_state_T = obj.left_leader_state_T;
            T = obj.time_horizon;
            dt = obj.dt;
            group_state = obj.group_state;
            CTS = CyberTrafficSystem(obj.traffic_settings);
            
            
            position = obj.calculate_positions(x);
            velocity = reshape(x, T+1, []);
            position = reshape(position, T+1, []);
            
            %右車線の制約
            right_leader_state_T = permute(right_leader_state_T, [3 2 1]);
            right_velocity = velocity(:, group_state(5, :, 1) == 0);
            right_position = position(:, group_state(5, :, 1) == 0);
            
            R = obj.right_vehicle_count;
            right_C = zeros(T, R);
            for n = 1:R
                if n == 1
                    xl_T = right_leader_state_T(1:T, :, 2);
                    vl_T = right_leader_state_T(1:T, :, 3);
                    
                else
                    xl_T = right_position(1:T, n-1);
                    vl_T = right_velocity(1:T, n-1);
                end
                
                xc_T = right_position(1:T, n);
                vc_T = right_velocity(1:T, n);
                
                right_C(:, n) = right_velocity(2:T+1, n) - CTS.car_following_model(xl_T, xc_T, vl_T, vc_T);
            end
            
            %左車線の制約
            left_leader_state_T = permute(left_leader_state_T, [3 2 1]);
            left_velocity = velocity(:, group_state(5, :, 1) == 1);
            left_position = position(:, group_state(5, :, 1) == 1);
            
            L = obj.left_vehicle_count;
            left_C = zeros(T, L);
            for n = 1:L
                if n == 1
                    xl_T = left_leader_state_T(1:T, :, 2);
                    vl_T = left_leader_state_T(1:T, :, 3);
                    
                else
                    xl_T = left_position(1:T, n-1);
                    vl_T = left_velocity(1:T, n-1);
                end
                
                xc_T = left_position(1:T, n);
                vc_T = left_velocity(1:T, n);
                
                left_C(:, n) = left_velocity(2:T+1, n) - CTS.car_following_model(xl_T, xc_T, vl_T, vc_T);
            end
            C = reshape([right_C left_C], [], 1);
            Ceq = [];
        end
        
        %最適化fminconを実行する
        function r = partial_optimize(obj)
            %変数
            group_state = obj.group_state;
            T = obj.time_horizon;
            N = obj.group_vehicle_count;
            dt = obj.dt;
            %目的関数
            fun = @obj.objective_function;
            %初期値
            x0 = obj.generate_x0();
            %線形制約条件
            %速度不等式
            
            nearly_eye = zeros(T, T+1);
            nearly_eye(:, 1:T) = -eye(T);
            nearly_eye(:, 2:T+1) = nearly_eye(:, 2:T+1) + eye(T);
            upper_A = kron(eye(N), nearly_eye);
            lower_A = kron(eye(N), -nearly_eye);
            A = [upper_A;lower_A];
            b = zeros(2*N*T, 1);
            b(1:N*T) = obj.Amax * dt;
            b(N*T+1:2*N*T) = -obj.Amin * dt;
            %等式制約条件
            Aeq = [];
            beq = [];
            %境界条件
            lb = obj.Vmin * ones(N*(T+1), 1);
            lb(1:T+1:N*(T+1)) = group_state(3, :);
            ub = obj.Vmax * ones(N*(T+1), 1);
            ub(1:T+1:N*(T+1)) = group_state(3, :);
            %非線形制約条件
            con = @obj.car_following_constraints;
            
            %options = optimoptions('fmincon','MaxFunctionEvaluations', 10000, 'MaxIterations', 1000, 'Algorithm', 'sqp', 'GradObj', 'on', 'CheckGradients', true, 'SpecifyObjectiveGradient', true, 'FiniteDifferenceType', 'central', 'FiniteDifferenceStepSize', 1e-15);
            options = optimoptions('fmincon','MaxFunctionEvaluations', 10000, 'Algorithm', 'sqp');
            %最適化を実行する
            r = fmincon(fun, x0, A, b, Aeq, beq, lb, ub, con, options);
        end
    end
end


%% 初期設定
close all
clear 

%シミュレーション変数
global car_following_model_type
global dt
global N

%車変数
global l
global gap_min
global Vmax
global Vdes
global Vmin
global Amax
global Ades
global tau

%環境変数
global road_length

%車追従モデルの選択
%1:クラウスモデル 2:IDMモデル
car_following_model_type = 2;

dt = 1;%単位時間
N = 12;

if car_following_model_type == 1
    Vdes = 22;
    Amax = 2.0; %最高加速度
    Ades = -4.5; %最低加速度
    tau = 3;%ドライバーの反応速度
elseif car_following_model_type == 2
    Vdes = 22;
    Amax = 0.73;
    Ades = -1.67;
    gap_min = 2;
end

l = 5; %車の長さ
Vmax = 27; %最高速度
Vmin = 0; %最低速度

road_length = 800;

%基本的な変数の設定
global T
global C
global n

global lane_change_vehicle_index
global next_lane_change_vehicle_index
global leader_group

global time_interval

global state_variable_i
global leader_lane_3

T = 10; %予測最適化ステップ数
C = 5; %実最適化区間ステップ数
n = 3; %一つのグループにおける車の数

sp = -50;%-65
er = -20;

vehicle_insert_interval = 4;

%状態変数の初期状態
initial_id = 1:N;
initial_position = [0 er sp sp+er 2*sp 2*sp+er 3*sp 3*sp+er 4*sp 4*sp+er 5*sp 5*sp+er];
initial_velocity = 16.6*ones(1, N);
initial_accel = NaN(1, N);
initial_lane = [0 1 0 1 0 1 0 1 0 1 0 1];
initial_lanechange = [1 1 0 0 1 0 1 0 0 0 0 1];
initial_cooldown = zeros(1, N);

insert_times = 8;
repetitions = 1;
simulation_time = 70;
random_position = -3 + 6*rand(insert_times*repetitions, N);
random_lanechange = randi([1 10], insert_times*repetitions, N);
random_lanechange(random_lanechange <= 3) = 1;
random_lanechange(random_lanechange > 3) = 0;

save('10 times random array', 'random_position', 'random_lanechange')
load('10 times random array', 'random_position', 'random_lanechange')

%fminconで用いる各々の変数設定
%x0は最適化する状態変数の初期値
x0 = zeros(1, 6*(T+1));
%xとvの関係式
Aeq = zeros(3*T, 6*(T+1));

Aeq(1:T, 1:T) = eye(T);
Aeq(T+1:2*T, T+2:2*(T+1)-1) = eye(T);
Aeq(2*T+1:3*T, 2*(T+1)+1:3*(T+1)-1) = eye(T);

Aeq(1:T, 2:T+1) = Aeq(1:T, 2:T+1) - eye(T);
Aeq(T+1:2*T, T+3:2*(T+1)) = Aeq(T+1:2*T, T+3:2*(T+1)) - eye(T);
Aeq(2*T+1:3*T, 2*(T+1)+2:3*(T+1)) = Aeq(2*T+1:3*T, 2*(T+1)+2:3*(T+1)) - eye(T);

Aeq(1:T, 3*(T+1)+2:4*(T+1)) = eye(T);
Aeq(T+1:2*T, 4*(T+1)+2:5*(T+1)) = eye(T);
Aeq(2*T+1:3*T, 5*(T+1)+2:6*(T+1)) = eye(T);

beq = zeros(1, 3*T);
%加速度の式
A = zeros(6*T, 6*(T+1));
A(1:T, 3*(T+1)+1:4*(T+1)-1) = -eye(T);
A(1:T, 3*(T+1)+2:4*(T+1)) = A(1:T, 3*(T+1)+2:4*(T+1)) + eye(T);
A(T+1:2*T, 3*(T+1)+1:4*(T+1)-1) = eye(T);
A(T+1:2*T, 3*(T+1)+2:4*(T+1)) = A(T+1:2*T, 3*(T+1)+2:4*(T+1)) - eye(T);

A(2*T+1:3*T, 4*(T+1)+1:5*(T+1)-1) = -eye(T);
A(2*T+1:3*T, 4*(T+1)+2:5*(T+1)) = A(2*T+1:3*T, 4*(T+1)+2:5*(T+1)) + eye(T);
A(3*T+1:4*T, 4*(T+1)+1:5*(T+1)-1) = eye(T);
A(3*T+1:4*T, 4*(T+1)+2:5*(T+1)) = A(3*T+1:4*T, 4*(T+1)+2:5*(T+1)) - eye(T);

A(4*T+1:5*T, 5*(T+1)+1:6*(T+1)-1) = -eye(T);
A(4*T+1:5*T, 5*(T+1)+2:6*(T+1)) = A(4*T+1:5*T, 5*(T+1)+2:6*(T+1)) + eye(T);
A(5*T+1:6*T, 5*(T+1)+1:6*(T+1)-1) = eye(T);
A(5*T+1:6*T, 5*(T+1)+2:6*(T+1)) = A(5*T+1:6*T, 5*(T+1)+2:6*(T+1)) - eye(T);

b = zeros(1, 6*T);

b(1:T) = Amax;
b(T+1:2*T) = -Ades;%-Ades
b(2*T+1:3*T) = Amax;
b(3*T+1:4*T) = -Ades;%-Ades
b(4*T+1:5*T) = Amax;
b(5*T+1:6*T) = -Ades;%-Ades


lb = zeros(1, 6*(T+1));
lb(1:3*(T+1)) = -Inf;
lb(3*(T+1)+1:end) = Vmin;

ub = zeros(1, 6*(T+1));
ub(1:3*(T+1)) = Inf;
ub(3*(T+1)+1:end) = Vmax;

human_deta = NaN(C*simulation_time+1, N*insert_times, 6, repetitions);
MPC_deta = NaN(C*simulation_time+1, N*insert_times, 6, repetitions);
tic
for r = 1:repetitions
    %% 人間の交通調整モデル
    state_variable = [];
    times = 1;
    
    for t = 0:C*simulation_time
        %車の列の挿入
        if rem(t, C*vehicle_insert_interval) == 0 && times <= insert_times
            add_state_variable = [initial_id + N*(times-1);initial_position + random_position(insert_times*(r-1)+times, :) - 200;initial_velocity;initial_accel;initial_lane;random_lanechange(insert_times*(r-1)+times, :);initial_cooldown];
            state_variable = cat(2, state_variable, add_state_variable);
            times = times + 1;
        end
        
        %車を追従させる
        leader_lane_3 = [];
        leader_group = [];
        state_variable = follow_car(state_variable, []);
        
        %車情報を記録する
        human_deta(t+1, state_variable(1, :), :, r) = state_variable(2:7, :).';
        
        %車線変更の実行
        state_variable = change_lane(state_variable, 2);
        
        %車を前の位置から並べ直す
        [B, index] = sort(state_variable(2, :), 'descend');
        state_variable = state_variable(:, index);
        
        %道路から出ていった車を排除する
        if state_variable(2, N) >= road_length + 200 && all(state_variable(6, 1:N) == 0)
            state_variable(:, 1:N) = [];
        end
        
        %道路から車がいなくなったとき、終了する
        if size(state_variable, 2) == 0
            break
        end
    end
    
    %% システムの交通調整モデル
    
    state_variable = [];
    times = 1;
    
    %実際の処理部分
    for time_interval = 0:simulation_time-1
        %車の列を挿入する
        if rem(time_interval, vehicle_insert_interval) == 0 && times <= insert_times
            add_state_variable = [initial_id + N*(times-1);initial_position + random_position(insert_times*(r-1)+times, :) - 200;initial_velocity;initial_accel;initial_lane;random_lanechange(insert_times*(r-1)+times, :);initial_cooldown];
            state_variable = cat(2, state_variable, add_state_variable);
            
            %MPC_detaに初期値を代入する
            MPC_deta(C*time_interval+1, state_variable(1, :), :, r) = state_variable(2:end, :)';
            
            times = times + 1;
        end
        
        %state_variableを3つのグループに分ける
        state_variable = reshape(state_variable, 7, 3, []);
        state_variable_A = state_variable(:, :, state_variable(2, 1, :) >= road_length - 200);
        state_variable_B = state_variable(:, :, state_variable(2, 3, :) >= -200 & state_variable(2, 1, :) < road_length - 200);
        state_variable_C = state_variable(:, :, state_variable(2, 3, :) < -200);
        
        state_variable_A = reshape(state_variable_A, 7, [], 1);
        state_variable_B = reshape(state_variable_B, 7, [], 1);
        state_variable_C = reshape(state_variable_C, 7, [], 1);
        
        %T秒後までの位置と速度の記録
        resultA = NaN(T+1, size(state_variable_A, 2), 6);
        resultB = NaN(T+1, size(state_variable_B, 2), 6);
        resultC = NaN(T+1, size(state_variable_C, 2), 6);
        
        leader_lane_3 = [];
        leader_group = [];
        
        %Aグループの処理
        if size(state_variable_A, 2) > 0
            state_variable_AA = state_variable_A;
            resultA(1, :, :) = state_variable_AA(2:7, :).';

            %車を前の位置から並べ直す
            [B, index] = sort(state_variable_AA(2, :), 'descend');
            state_variable_AAA = state_variable_AA(:, index);
            %整理する
            state_variable_ii = arrange(state_variable_AAA(:, end-2:end)); 
            %leader_lane_33
            leader_lane_33 = zeros(1, T+1);
            leader_lane_33(1) = state_variable_ii(5, 3);
            %leader_group_123
            leader_group_123 = zeros(T+1, 3, 2);
            leader_group_123(1, :, :) = state_variable_ii(2:3, :).';
            
            for t = 1:T
                %車を追従させる
                state_variable_AA = follow_car(state_variable_AA, t+1);
                resultA(t+1, :, :) = state_variable_AA(2:7, :).';
                if t == C
                    state_variable_A = state_variable_AA;
                end
                
                %車を前の位置から並べ直す
                [B, index] = sort(state_variable_AA(2, :), 'descend');
                state_variable_AAA = state_variable_AA(:, index);
                %整理する
                state_variable_ii = arrange(state_variable_AAA(:, end-2:end));
                %leader_lane_33
                leader_lane_33(t+1) = state_variable_ii(5, 3);
                %leader_group_123
                leader_group_123(t+1, :, :) = state_variable_ii(2:3, :).';
                %車線変更の実行
                state_variable_AA = change_lane(state_variable_AA, 2);
                %最後のグループだけ車線変更しない
                %最後のグループのindexを検索
                index = [0 0 0];
                state_variable_AAA = state_variable_AAA(:, end-2:end);
                for i = 1:3
                    index(i) = find(state_variable_AA(1, :) == state_variable_AAA(1, i));
                end
                
                state_variable_AA(:, index) = state_variable_AAA;
            end
            
            leader_lane_3 = leader_lane_33;
            leader_group = reshape(leader_group_123, 1, []);

        end
        
        %Bグループの処理
        if size(state_variable_B, 2) > 0
            %state_variable_Bを三次元配列に直す
            state_variable_B = reshape(state_variable_B, 7, 3, []);
            for group = 1:size(state_variable_B, 3)
                state_variable_i = state_variable_B(:, :, group);
                %車を並べ替える
                state_variable_i = arrange(state_variable_i);
                %初期の配列の順番を記録する
                original_state_variable_i = state_variable_i;
                
                %車線変更する車を決める
                if state_variable_i(2, 3) < -200
                    lane_change_vehicle_index = [];
                    next_lane_change_vehicle_index = [];
                else
                    if isequal(state_variable_i(6, :), [0 0 0])
                        lane_change_vehicle_index = [];
                        next_lane_change_vehicle_index = [];
                        
                    elseif isequal(state_variable_i(6, :), [0 1 1])
                        lane_change_vehicle_index = [2 3];
                        next_lane_change_vehicle_index = [];
                        
                    elseif isequal(state_variable_i(6, :), [1 0 0])
                        lane_change_vehicle_index = 1;
                        next_lane_change_vehicle_index = [];
                        
                    elseif isequal(state_variable_i(6, :), [1 1 0])
                        lane_change_vehicle_index = [1 2];%2
                        next_lane_change_vehicle_index = [];%1
                        
                    elseif isequal(state_variable_i(6, :), [0 1 0])
                        lane_change_vehicle_index = 2;
                        next_lane_change_vehicle_index = [];
                        
                    elseif isequal(state_variable_i(6, :), [1 1 1])
                        lane_change_vehicle_index = [1 2 3];%3
                        next_lane_change_vehicle_index = [];%2
                        
                    elseif isequal(state_variable_i(6, :), [1 0 1])
                        lane_change_vehicle_index = [1 3];%3
                        next_lane_change_vehicle_index = [];%1
                        
                    elseif isequal(state_variable_i(6, :), [0 0 1])
                        lane_change_vehicle_index = 3;
                        next_lane_change_vehicle_index = [];
                    end
                end
                
                %初期位置、初期速度を制約に入れる
                lb(1) = state_variable_i(2, 1);
                ub(1) = state_variable_i(2, 1);
                lb((T+1)+1) = state_variable_i(2, 2);
                ub((T+1)+1) = state_variable_i(2, 2);
                lb(2*(T+1)+1) = state_variable_i(2, 3);
                ub(2*(T+1)+1) = state_variable_i(2, 3);
                
                lb(3*(T+1)+1) = state_variable_i(3, 1);
                ub(3*(T+1)+1) = state_variable_i(3, 1);
                lb(4*(T+1)+1) = state_variable_i(3, 2);
                ub(4*(T+1)+1) = state_variable_i(3, 2);
                lb(5*(T+1)+1) = state_variable_i(3, 3);
                ub(5*(T+1)+1) = state_variable_i(3, 3);
                
                %初期値x0を決定する
                x0 = generate_x0();
                
                %最適化fminconを実行する
                options = optimoptions('fmincon', 'MaxFunctionEvaluations', 10000, 'MaxIterations', 1000, 'Algorithm', 'sqp', 'GradObj','on');
                result = fmincon(@new_lane_change_function, x0, A, b, Aeq, beq, lb, ub, @car_following_constraints_3_vehicles, options);
                
                %各々の車線の最後尾車両を記憶する
                leader_group = result;

                %最適化結果を入力する
                state_variable_i(2, 1) = result(C+1);
                state_variable_i(2, 2) = result((T+1)+C+1);
                state_variable_i(2, 3) = result(2*(T+1)+C+1);
                state_variable_i(3, 1) = result(3*(T+1)+C+1);
                state_variable_i(3, 2) = result(4*(T+1)+C+1);
                state_variable_i(3, 3) = result(5*(T+1)+C+1);
                state_variable_i(4, 1) = result(3*(T+1)+C+1) - result(3*(T+1)+C);
                state_variable_i(4, 2) = result(4*(T+1)+C+1) - result(4*(T+1)+C);
                state_variable_i(4, 3) = result(5*(T+1)+C+1) - result(5*(T+1)+C);
                
                %次のグループのために今のグループの情報を記録する
                leader_lane_3(1:T+1) = state_variable_i(5, 3);
                
                %順番をもとに戻して更新
                [B, index] = sort(original_state_variable_i(2, :), 'descend');
                state_variable_i = state_variable_i(:, index);
                state_variable_B(:, :, group) = state_variable_i;
                
                result = reshape(result, T+1, n, 2);
                result = result(:, index, :);
                result(:, :, 3) = state_variable_i(4, :).*ones(T+1, n);
                result(:, :, 4) = state_variable_i(5, :).*ones(T+1, n);
                result(:, :, 5) = state_variable_i(6, :).*ones(T+1, n);
                result(:, :, 6) = state_variable_i(7, :).*ones(T+1, n);
                resultB(:, n*(group-1)+1:n*group, 1) = result(:, :, 1);
                resultB(:, n*(group-1)+1:n*group, 2) = result(:, :, 2);
                resultB(:, n*(group-1)+1:n*group, 3) = result(:, :, 3);
                resultB(:, n*(group-1)+1:n*group, 4) = result(:, :, 4);
                resultB(:, n*(group-1)+1:n*group, 5) = result(:, :, 5);
                resultB(:, n*(group-1)+1:n*group, 6) = result(:, :, 6);
                
            end
            %state_variable_Bを二次元配列に戻す
            state_variable_B  = reshape(state_variable_B, 7, [], 1);
            
            %車線変更の実行
            state_variable_B = change_lane(state_variable_B, 2);

        end
        
        %Cグループの処理
        if size(state_variable_C, 2) > 0
            state_variable_CC = state_variable_C;
            resultC(1, :, :) = state_variable_CC(2:7, :).';
            for t = 1:T
                %車を追従させる
                state_variable_CC = follow_car(state_variable_CC, t+1);
                resultC(t+1, :, :) = state_variable_CC(2:7, :).';
                if t == C
                    state_variable_C = state_variable_CC;
                end
                %車線変更の実行
                state_variable_CC = change_lane(state_variable_CC, 2);
            end
        end
        
        %結果を入力する
        result3 = cat(2, resultA, resultB, resultC);
        state_variable = cat(2, state_variable_A, state_variable_B, state_variable_C);
        
        %車の情報を記録する  
        MPC_deta(C*time_interval+2:C*(time_interval+1)+1, state_variable(1, :), :, r) = result3(2:C+1, :, :);
        %車を前の位置から並べ直す
        [B, index] = sort(state_variable(2, :), 'descend');
        state_variable = state_variable(:, index);
        %道路から出ていった車を排除する
        if state_variable(2, N) >= road_length + 200 && all(state_variable(6, 1:N) == 0)
            state_variable(:, 1:N) = [];
        end
        
        %道路から車がいなくなった時、終了する
        if size(state_variable, 2) == 0
            break
        end
    end
end

toc
%% 車情報をファイルに保存する
save('12 vehicles per 35 seconds', 'dt', 'road_length', 'human_deta', 'MPC_deta');


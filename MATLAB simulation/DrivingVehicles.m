% 走行する車郡のオブジェクト
% その他の情報(例えば、道路の情報やシミュレーションの情報は含まない)
classdef DrivingVehicles
    properties(SetAccess = private, GetAccess = public)
        %車情報が入った行列
        driving_vehicles
    end
    
    properties(Access = private)
        %マジックナンバーをなくす
        id_index
        position_index
        velocity_index
        acceleration_index
        lane_index
        lanechange_request_index
        lanechange_cooldown_index
        
        %行列の軸がそれぞれなにを意味するかを示す
        aspect_axis
        vehicle_axis
    end
    
    methods
        %コンストラクタ
        function obj = DrivingVehicles(matrix)
            obj.driving_vehicles = matrix;
            
            %定数
            obj.id_index = 1;
            obj.position_index = 2;
            obj.velocity_index = 3;
            obj.acceleration_index = 4;
            obj.lane_index = 5;
            obj.lanechange_request_index = 6;
            obj.lanechange_cooldown_index = 7;
            
            obj.aspect_axis = 1;
            obj.vehicle_axis = 2;
        end
        
        %車を分類する
        %右車線の車を取り出す
        function r = get_rightlane_vehicles(obj)
            driving_vehicles = obj.driving_vehicles(:, obj.driving_vehicles(obj.lane_index, :) == 0);
            r = DrivingVehicles(driving_vehicles);%クラスインスタンスとして再生成する
        end
        
        %左車線の車を取り出す
        function r = get_leftlane_vehicles(obj)
            driving_vehicles = obj.driving_vehicles(:, obj.driving_vehicles(obj.lane_index, :) == 1);
            r = DrivingVehicles(driving_vehicles);%クラスインスタンスとして再生成する
        end
        
        %右車線の末尾の車を取り出す
        function r = get_rightlane_endvehicle(obj)
            rightlane_vehicles = obj.get_rightlane_vehicles();
            if size(rightlane_vehicles, obj.vehicle_axis) > 0 
                rightlane_endvehicles = rightlane_vehicles(:, end);
            else
                rightlane_endvehicles = rightlane_vehicles(:, rightlane_vehicles(obj.lane_index, :) == 2);% つまり[]
            end 
            r = DrivingVehicles(rightlane_endvehicles);%クラスインスタンスとして再生成する
        end
        
        %左車線の末尾の車を取り出す
        function r = get_leftlane_endvehicle(obj)
            leftlane_vehicles = obj.get_leftlane_vehicles();
            if size(leftlane_vehicles, obj.vehicle_axis) > 0
                leftlane_endvehicles = leftlane_vehicles(:, end);
            else
                leftlane_endvehicles = leftlane_vehicles(:, leftlane_vehicles(obj.lane_index, :) == 2);% つまり[]
            end
            r = DrivingVehicles(leftlane_endvehicles);
        end
        
        %車の性質の参照
        %車の番号を取得する
        function r = see_id(obj)
            r = obj.driving_vehicles(id_list_index, :);
        end
        
        function r = see_position(obj)
            r = obj.driving_vehicles(position_index, :);
        end
        
        function r = see_velocity(obj)
            r = obj.driving_vehicles(velocity_index, :);
        end
        
        function r = see_acceleration(obj)
            r = obj.driving_vehicles(obj.acceleration_index, :);
        end
        
        function r = see_lane(obj)
            r = obj.driving_vehicles(obj.lane_index, :);
        end
        
        function r = see_lanechange_request(obj)
            r = obj.driving_vehicles(obj.lanechange_request_index, :);
        end
        
        function r = see_lanechange_cooldown(obj)
            r = obj.driving_vehicles(obj.lanechange_cooldown_index, :);
        end
        
        %新しい車を挿入する
        function r = insert(obj, vehicles)
            driving_vehicles = cat(obj.vehicle_axis, obj.driving_vehicles, vehicles);
            r = DrivingVehicles(driving_vehicles);
        end
        
    end
end
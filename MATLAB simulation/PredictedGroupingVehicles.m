classdef PredictedGroupingVehicles
    properties(SetAccess = private, GetAccess = public)
        %車情報が入った行列
        predicted_grouping_vehicles
    end
    
    properties(Access = private)
        id_index
        position_index
        velocity_index
        acceleration_index
        lane_index
        lanechange_request_index
        lanechange_cooldown_index
        
        aspect_axis
        vehicle_axis
        time_axis
    end
    
    methods
        %コンストラクタ
        function obj = PredictedGroupingVehicles(matrix)
            obj.predicted_grouping_vehicles = matrix;
            
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
            obj.time_axis = 3;
        end
        
        %車を分類する
        %右車線の車を取り出す
        function r = get_rightlane_vehicles(obj)
            matrix = obj.predicted_grouping_vehicles(:, obj.predicted_grouping_vehicles(obj.lane_index, :, 1) == 0, :);
            r = PredictedGroupingVehicles(matrix);
        end
        
        %左車線の車を取り出す
        function r = get_leftlane_vehicles(obj)
            matrix = obj.predicted_grouping_vehicles(:, obj.predicted_grouping_vehicles(obj.lane_index, :, 1) == 1, :);
            r = PredictedGroupingVehicles(matrix);
        end
        
        %右車線の末尾の車を取り出す
        function r = get_rightlane_endvehicle(obj)
            rightlane_vehicles = obj.get_rightlane_vehicles();
            if size(rightlane_vehicles, obj.vehicle_axis) > 0 
                rightlane_endvehicles = rightlane_vehicles(:, end, :);
            else
                rightlane_endvehicles = rightlane_vehicles(:, rightlane_vehicles(obj.lane_index, :) == 2, :);% つまり[]
            end
            r = PredictedGroupingVehicles(rightlane_endvehicles);
        end
        
        %左車線の末尾の車を取り出す
        function r = get_leftlane_endvehicle(obj)
            leftlane_vehicles = obj.get_leftlane_vehicles();
            if size(leftlane_vehicles, obj.vehicle_axis) > 0
                leftlane_endvehicles = leftlane_vehicles(:, end, :);
            else
                leftlane_endvehicles = leftlane_vehicles(:, leftlane_vehicles(obj.lane_index, :) == 2, :);% つまり[]
            end
            r = PredictedGroupingVehicles(leftlane_endvehicles);
        end
        
        %車の性質
        %車の番号を取得する
        function r = see_id(obj)
            r = obj.predicted_grouping_vehicles(obj.id_index, :, :);
        end
        
        function r = see_position(obj)
            r = obj.predicted_grouping_vehicles(obj.position_index, :, :);
        end
        
        function r = see_velocity(obj)
            r = obj.predicted_grouping_vehicles(obj.velocity_index, :, :);
        end
        
        function r = see_acceleration(obj)
            r = obj.predicted_grouping_vehicles(obj.acceleration_index, :, :);
        end
        
        function r = see_lane(obj)
            r = obj.predicted_grouping_vehicles(obj.lane_index, :, :);
        end
        
        function r = see_lanechange_request(obj)
            r = obj.predicted_grouping_vehicles(obj.lanechange_request_index, :, :);
        end
        
        function r = see_lanechange_cooldown(obj)
            r = obj.predicted_grouping_vehicles(obj.lanechange_cooldown_index, :, :);
        end
        
        function r = insert(obj, vehicles)
            predicted_grouping_vehicles = cat(obj.vehicle_axis, obj.predicted_grouping_vehicles, vehicles);
            r = PredictedGroupingVehicles(predicted_grouping_vehicles);
        end
    end
end
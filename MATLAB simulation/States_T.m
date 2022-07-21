classdef States_T
    properties
        state_T
    end
    
    methods
        %コンストラクタ
        function obj = States_T(init_value)
            obj.state_T = init_value;
        end
        
        %車を分類する
        %右車線の車を取り出す
        function r = right_state_T(obj)
            r = obj.state_T(:, obj.state(5, :, 1) == 0, :);
        end
        
        %右車線の末尾の車を取り出す
        function r = right_tail_state_T(obj)
            right_state_T = obj.right_state_T();
            if size(right_state_T, 2) > 0 
                r = right_state_T(:, end, :);
            else
                r = right_state_T(:, right_state_T(5, :) == 2, :);% つまり[]
            end
        end
        
        %左車線の車を取り出す
        function r = left_state_T(obj)
            r = obj.state_T(:, obj.state_T(5, :) == 1, :);
        end
        
        %左車線の末尾の車を取り出す
        function r = left_tail_state_T(obj)
            left_state_T = obj.left_state_T();
            if size(left_state_T, 2) > 0
                r = left_state_T(:, end);
            else
                r = left_state_T(:, left_state_T(5, :) == 2);% つまり[]
            end
        end
        
        %車の性質
        %車の番号を取得する
        function r = id_list(obj)
            r = obj.state_T(1, :, :);
        end
        
        function r = position(obj)
            r = obj.state_T(2, :, :);
        end
        
        function r = velocity(obj)
            r = obj.state_T(3, :, :);
        end
        
        function r = acceleration(obj)
            r = obj.state_T(4, :, :);
        end
        
        function r = lane(obj)
            r = obj.state_T(5, :, :);
        end
        
        function r = will_change_lanes(obj)
            r = obj.state_T(6, :, :);
        end
        
        function r = lanechange_cooldown(obj)
            r = obj.state_T(7, :, :);
        end
    end
end
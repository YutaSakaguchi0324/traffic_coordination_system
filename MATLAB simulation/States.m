classdef States
    properties
        state
    end
    
    methods
        %コンストラクタ
        function obj = States(init_value)
            obj.state = init_value;
        end
        
        %車を分類する
        %右車線の車を取り出す
        function r = right_state(obj)
            r = obj.state(:, obj.state(5, :) == 0);
        end
        
        %右車線の末尾の車を取り出す
        function r = right_tail_state(obj)
            right_state = obj.right_state();
            if size(right_state, 2) > 0 
                r = right_state(:, end);
            else
                r = right_state(:, right_state(5, :) == 2);% つまり[]
            end
        end
        
        %左車線の車を取り出す
        function r = left_state(obj)
            r = obj.state(:, obj.state(5, :) == 1);
        end
        
        %左車線の末尾の車を取り出す
        function r = left_tail_state(obj)
            left_state = obj.left_state();
            if size(left_state, 2) > 0
                r = left_state(:, end);
            else
                r = left_state(:, left_state(5, :) == 2);% つまり[]
            end
        end
        
        %車の性質
        %車の番号を取得する
        function r = id_list(obj)
            r = obj.state(1, :);
        end
        
        function r = position(obj)
            r = obj.state(2, :);
        end
        
        function r = velocity(obj)
            r = obj.state(3, :);
        end
        
        function r = acceleration(obj)
            r = obj.state(4, :);
        end
        
        function r = lane(obj)
            r = obj.state(5, :);
        end
        
        function r = will_change_lanes(obj)
            r = obj.state(6, :);
        end
        
        function r = lanechange_cooldown(obj)
            r = obj.state(7, :);
        end
    end
end
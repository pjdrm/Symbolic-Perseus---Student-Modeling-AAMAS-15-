function [nextState, obs] = studentAVL(action, stateConfig, studentConfig)
    nextState = stateConfig;
    if action == 1 & ~isempty(strfind(studentConfig, 'sk_height_empty'))
        nextState(2, 1) = 1;
    end
    
    if action == 2 & ~isempty(strfind(studentConfig, 'sk_height_leaf'))
        nextState(2, 2) = 1;
    end
    
    if action == 3 & ~isempty(strfind(studentConfig, 'sk_height_tree'))
        nextState(2, 3) = 1;
    end
    
    if action == 4 & ~isempty(strfind(studentConfig, 'sk_calc_bf'))
        nextState(2, 4) = 1;
    end
    
    if action == 1 | action == 2 | action == 3 | action == 4
        obs = [12;3];
    elseif all(stateConfig(2, :) == 1)
        obs = [12;1];
    else
        obs = [12;2];
    end
end
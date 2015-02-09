function [minStepsArray, performedStepsArray] = evalDefaultpolicyAVL()
% function avRew = evalPOMDPpolicyStationary(pomdpFileName, policyFileName, nRuns, nSteps, initStateConfig)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Pascal Poupart (Copyright 2007)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inputs:
%
% pomdpFileName: file containing the POMDP description
% policyFileName: file containing the policy
% nRuns: number of policy runs (default value: 500)
% nSteps: number of time steps in each run (default value: 50)
% initStateConfig: initial state (default value: state sampled from initial belief state)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Outputs:
%
% avRew: expected total discounted reward averaged over all runs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Comments:
%
% Computes the expected total discounted reward averaged over all runs for
% the *stationary* greedy policy induced by a value function.  At each step,
% the action of the best alphaVector for the current belief state is
% executed.
%
% The average value computed may be arbitrarily lower than that of the 
% policy graph induced by boundedPerseus.  The value of the initial belief 
% state printed on the screen by boundedPerseus after each iteration is an 
% estimate of the value of the induced policy graph (not the stationary 
% policy).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%stateConfigs = [1 1 2 2 1; 2 2 2 2 1; 1 1 1 1 1];
%studentConfigs = {'sk_calc_bf sk_height_tree'; 'sk_height_empty sk_height_leaf sk_height_tree sk_calc_bf'; 'none'};
%studentIndexes = [2 3];
stateConfigs = [1 1 1 2 1; 1 2 2 1 1; 1 1 1 1 1; 1 1 1 1 1; 1 1 1 1 1; 2 2 2 2 1; 1 1 1 1 1; 1 1 1 1 1; 1 1 1 1 1; 1 1 1 1 1; 1 2 2 2 1; 1 1 1 1 1; 1 1 1 1 1; 1 1 1 1 1; 1 1 1 1 1; 2 2 2 2 1; 1 1 1 1 1; 1 1 1 1 1; 1 1 1 1 1; 1 1 1 1 1; 2 1 1 1 1; 2 2 2 2 1; 1 1 1 1 1; 1 1 1 1 1; 1 1 1 2 1; 1 1 1 1 1; 1 1 1 1 1; 1 1 1 2 1; 1 1 1 1 1; 1 1 1 1 1; 1 1 1 1 1; 1 1 1 1 1; 2 1 1 2 1; 1 1 1 1 1; 1 1 1 1 1; 1 1 1 1 1; 1 1 1 1 1];
studentConfigs = {'sk_calc_bf'; 'sk_height_leaf sk_height_tree'; 'none'; 'none'; 'none'; 'sk_height_leaf sk_calc_bf sk_height_tree sk_height_empty'; 'none'; 'none'; 'none'; 'none'; 'sk_height_leaf sk_height_tree sk_calc_bf'; 'none'; 'none'; 'none'; 'none'; 'sk_height_leaf sk_calc_bf sk_height_tree sk_height_empty'; 'none'; 'none'; 'none'; 'none'; 'sk_height_empty'; 'sk_height_leaf sk_calc_bf sk_height_tree sk_height_empty'; 'none'; 'none'; 'sk_calc_bf'; 'none'; 'none'; 'sk_calc_bf'; 'none'; 'none'; 'none'; 'none'; 'sk_calc_bf sk_height_empty'; 'none'; 'none'; 'none'; 'none'};
studentIndexes = [5 10 15 19 24 27 32 37];
minStepsTotal = 0;
performedStepsTotal = 0;
minStepsArray = zeros(1, size(studentIndexes, 2));
performedStepsArray = zeros(1, size(studentIndexes, 2));
i = 1;
performedStepsCurrentStudent = 0;
minStepsCurrentStudent = 0;

prevStudent = 1;
defaultPolicyActions = [5 4 5 3 5 1 2 5];
actionIndex = 1;
for currentStudent = studentIndexes
    for runId = prevStudent:currentStudent

      runId = runId
      Global.random.setSeed(runId);
      stateConfig = [1 2 3 4 5; stateConfigs(runId, :)];
      studentConfig = studentConfigs(runId, :);
      
      if strcmp(studentConfig, 'none')
          minStepsCurrentStudent = minStepsCurrentStudent + 1;
      else
        minStepsCurrentStudent = minStepsCurrentStudent + cellfun(@length,strfind(studentConfig, ' ')) + 2;
      end
      
      while 1
        %fprintf('stepId = %i\n',stepId);

        % pick action
        actId =  defaultPolicyActions(actionIndex);
        actionIndex = actionIndex + 1;
        if actionIndex > size(defaultPolicyActions, 2)
            actionIndex = 1;
        end
        
        performedStepsCurrentStudent = performedStepsCurrentStudent + 1;
        [nextStateConfig, obsConfig] = studentAVL(actId, stateConfig, studentConfig);
        if obsConfig(2) == 1
            break;
        end

        % update belief state
        stateConfig = nextStateConfig;

        Global.newHashtables;
        %keyboard
      end
      %keyboard
    end
    minStepsArray(i) = minStepsCurrentStudent;
    minStepsTotal = minStepsTotal + minStepsCurrentStudent;
    performedStepsArray(i) = performedStepsCurrentStudent;
    performedStepsTotal = performedStepsTotal + performedStepsCurrentStudent;
    i = i+1;
    minStepsCurrentStudent = 0;
    performedStepsCurrentStudent = 0;
    
    prevStudent = currentStudent+1;
end
fprintf('minStepsTotal = %f\t performedStepsTotal = %f\n', minStepsTotal, performedStepsTotal);


function [minStepsArray, performedStepsArray] = evalPOMDPpolicyAVL(pomdpFileName, policyFileName, nRuns, nSteps, initStateConfig)
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

% load POMDP problem
fprintf('Loading policy...\n');
POMDP = ParseSPUDD(pomdpFileName);
POMDP.parsePOMDP(false);
ddPOMDP = pomdpFlatAct(POMDP);

stateVars = 1:ddPOMDP.nStateVars;
observedVars = stateVars;
stateVarsPrime = stateVars+ddPOMDP.nVars;
obsVars = ddPOMDP.nStateVars+1:ddPOMDP.nVars;
obsVarsPrime = obsVars+ddPOMDP.nVars;

% load POMDP policy
load(policyFileName);
ddPOMDP = pomdpFlatAct(POMDP);

%[valueFunction, policy] = solveQMDP(ddPOMDP);

if ~exist('nRuns','var') | isempty(nRuns)
  nRuns = 500;
end

if ~exist('nSteps','var') | isempty(nSteps)
  nSteps = 50;
end

avRew = 0;
%stateConfigs = [1 1 2 2 1; 2 2 2 2 1; 1 1 1 1 1];
%studentConfigs = {'sk_calc_bf sk_height_tree'; 'sk_height_empty sk_height_leaf sk_height_tree sk_calc_bf'; 'none'};
%studentIndexes = [2 3];
stateConfigs = [1 1 1 2 1; 1 2 2 1 1; 1 1 1 1 1; 1 1 1 1 1; 1 1 1 1 1; 2 2 2 2 1; 1 1 1 1 1; 1 1 1 1 1; 1 1 1 1 1; 1 1 1 1 1; 1 2 2 2 1; 1 1 1 1 1; 1 1 1 1 1; 1 1 1 1 1; 1 1 1 1 1; 2 2 2 2 1; 1 1 1 1 1; 1 1 1 1 1; 1 1 1 1 1; 1 1 1 1 1; 2 1 1 1 1; 2 2 2 2 1; 1 1 1 1 1; 1 1 1 1 1; 1 1 1 2 1; 1 1 1 1 1; 1 1 1 1 1; 1 1 1 2 1; 1 1 1 1 1; 1 1 1 1 1; 1 1 1 1 1; 1 1 1 1 1; 2 1 1 2 1; 1 1 1 1 1; 1 1 1 1 1; 1 1 1 1 1; 1 1 1 1 1];
studentConfigs = {'sk_calc_bf'; 'sk_height_leaf sk_height_tree'; 'none'; 'none'; 'none'; 'sk_height_leaf sk_calc_bf sk_height_tree sk_height_empty'; 'none'; 'none'; 'none'; 'none'; 'sk_height_leaf sk_height_tree sk_calc_bf'; 'none'; 'none'; 'none'; 'none'; 'sk_height_leaf sk_calc_bf sk_height_tree sk_height_empty'; 'none'; 'none'; 'none'; 'none'; 'sk_height_empty'; 'sk_height_leaf sk_calc_bf sk_height_tree sk_height_empty'; 'none'; 'none'; 'sk_calc_bf'; 'none'; 'none'; 'sk_calc_bf'; 'none'; 'none'; 'none'; 'none'; 'sk_calc_bf sk_height_empty'; 'none'; 'none'; 'none'; 'none'};
studentIndexes = [5 10 15 19 24 27 32 37];
minStepsTotal = 0;
performedStepsTotal = 0;
prevStudent = 1;

minStepsArray = zeros(1, size(studentIndexes, 2));
performedStepsArray = zeros(1, size(studentIndexes, 2));
i = 1;
performedStepsCurrentStudent = 0;
minStepsCurrentStudent = 0;
strActions = '';

for currentStudent = studentIndexes
    belState = ddPOMDP.initialBelState;
    for runId = prevStudent:currentStudent

      runId = runId
      rand('state',runId);
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
        [actId,alphaRatings] =  policyQuery(stateVars,belState,valueFunction,policy);
        
        %debug
        debug = 'POMDP'
        stateConfig
        ddPOMDP.actions(actId).name
        strActions = [strActions, ' ', ddPOMDP.actions(actId).name];
        
        performedStepsCurrentStudent = performedStepsCurrentStudent + 1;
        [nextStateConfig, obsConfig] = studentAVL(actId, stateConfig, studentConfig);
        if obsConfig(2) == 1
            debug = 'new student'
            break;
        end

        % update belief state
        belState = beliefUpdate(ddPOMDP, belState, actId, obsConfig);
        stateConfig = nextStateConfig;

        Global.newHashtables;
        %keyboard
      end
      strActions = strcat(strActions, '\n');
      %keyboard
    end
    strActions = strcat(strActions, '------------------\n');
    minStepsArray(i) = minStepsCurrentStudent;
    minStepsTotal = minStepsTotal + minStepsCurrentStudent;
    performedStepsArray(i) = performedStepsCurrentStudent;
    performedStepsTotal = performedStepsTotal + performedStepsCurrentStudent;
    i = i+1;
    minStepsCurrentStudent = 0;
    performedStepsCurrentStudent = 0;
    
    prevStudent = currentStudent+1;
end
fid = fopen('./debug.txt','wt');
fprintf(fid,'%s', sprintf(strActions));
fclose(fid);  
fprintf('minStepsTotal = %f\t performedStepsTotal = %f\n',minStepsTotal,performedStepsTotal);


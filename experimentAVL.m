function experimentAVL(problemFile)
[valueFunction,policy, policyFileName] = solvePOMDP(problemFile);
[minSteps, performedStepsPOMDP] = evalPOMDPpolicyAVL(problemFile, policyFileName);
[minSteps, performedStepsDefault] = evalDefaultpolicyAVL();
figure(1);
hold on;
bar([minSteps' performedStepsPOMDP' performedStepsDefault'], 'grouped');
%plot(minSteps);
%plot(performedStepsPOMDP, 'g');
%plot(performedStepsDefault, 'r');
%axis([1 size(minSteps, 2) 0 max([performedStepsPOMDP performedStepsDefault minSteps])+2])
legend('Optimal','POMDP', 'Baseline')
xlabel('Students');
ylabel('Number of actions');
hold off;
sum(performedStepsPOMDP)


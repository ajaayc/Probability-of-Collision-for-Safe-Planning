figure(1);
M = csvread('SimulationHistogramProp.csv')
h = histogram(M);
h.BinLimits = [min(M) 1];
h.BinEdges = min(M):.015:1
h.Normalization = 'probability'
title({'Proportions of Particles Colliding for Monte Carlo Simulations','(100 Simulations, 10,000 particles)'})
xlabel('Proportion of Particles Colliding')
ylabel('Normalized Frequency')
mean(M)
std(M)
min(M)
max(M)
figure(2);
M2 = csvread('SimulationHistogramTimes.csv')
h2 = histogram(M2);
%h2.BinLimits = [min(M2) max(M2)];
%h2.BinEdges = min(M2):2:max(M2)
h2.Normalization = 'probability'
title({'Time Required for Monte Carlo Simulations','(100 Simulations, 10,000 particles)'})
xlabel('Simulation Time(sec)')
ylabel('Normalized Frequency')
mean(M2)
std(M2)
min(M2)
max(M2)
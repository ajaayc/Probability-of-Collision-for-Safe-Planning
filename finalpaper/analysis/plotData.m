function plotData(propfile,timefile,title1,title2,f1,f2)
figure(f1);
M = csvread(propfile);
h = histogram(M);
h.BinLimits = [min(M) 1];
h.BinEdges = min(M):.015:1;
h.Normalization = 'probability';
title(title1)
xlabel('Probability of Collision')
ylabel('Normalized Frequency')
disp('Mean')
mean(M)
disp('std')
std(M)
disp('min')
min(M)
disp('max')
max(M)
figure(f2);
M2 = csvread(timefile);
h2 = histogram(M2);
%h2.BinLimits = [min(M2) max(M2)];
%h2.BinEdges = min(M2):2:max(M2)
h2.Normalization = 'probability';
title(title2);
xlabel('Time Required(sec)')
ylabel('Normalized Frequency')
disp('Mean')
mean(M2)
disp('std')
std(M2)
disp('min')
min(M2)
disp('max')
max(M2)
end
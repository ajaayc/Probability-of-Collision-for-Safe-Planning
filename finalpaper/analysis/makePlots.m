disp('MC')
plotData('MC_prop.csv','MC_time.csv',{'Proportions of Particles Colliding for Monte Carlo Simulations','(200 Simulations, 10,000 particles)'},{'Time Required for Monte Carlo Simulations','(200 Simulations, 10,000 particles)'},1,2)
disp('GMM1')
plotData('GMM1_prop.csv','GMM1_time.csv',{'Probabilities of Collision for 1-component GMM','(200 Executions, 10,000 samples)'},{'Time Required for 1-Component GMM','(200 Executions, 10,000 particles)'},3,4)
disp('GMM2')
plotData('GMM2_prop.csv','GMM2_time.csv',{'Probabilities of Collision for 2-component GMM','(200 Executions, 10,000 samples)'},{'Time Required for 2-Component GMM','(200 Executions, 10,000 particles)'},5,6)
disp('GMM3')
plotData('GMM3_prop.csv','GMM3_time.csv',{'Probabilities of Collision for 3-component GMM','(200 Executions, 10,000 samples)'},{'Time Required for 3-Component GMM','(200 Executions, 10,000 particles)'},7,8)
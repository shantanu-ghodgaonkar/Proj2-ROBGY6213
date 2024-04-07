function plotData(estimatedV, data, Vicon, Time, datasetNum, ransacFlag)
%PLOTDATA Plot the predicted data
xVel = estimatedV(1,:);
yVel = estimatedV(2,:);
zVel = estimatedV(3,:);
xAngVel = estimatedV(4,:);
yAngVel = estimatedV(5,:);
zAngVel = estimatedV(6,:);
sampledTime = vertcat(data(:).t);

dnum = ['dataset', int2str(datasetNum)];
if ransacFlag == 1
    path = 'I:/My Drive/Academics/NYUMSMRSPRING24/Robot Localization and Navigation (ROB-GY 6213)/Project/Project2/LaTeX/img/plots/part2/wRANSAC/';
    figure('Name', sprintf('Part %d (w/ RANSAC)- Dataset %d', 2, datasetNum));
else 
    path = 'I:/My Drive/Academics/NYUMSMRSPRING24/Robot Localization and Navigation (ROB-GY 6213)/Project/Project2/LaTeX/img/plots/part2/woRANSAC/';
    figure('Name', sprintf('Part %d (w/o RANSAC)- Dataset %d', 2, datasetNum));
end
%%

plot(sampledTime, xVel,'r', Time, Vicon(7,:),'b');
title('Velocity X');
legend('Predicted', 'Actual');
saveas(gcf, [path, dnum, '/Velocity X'], 'png');
plot(sampledTime, xAngVel, 'r',Time, Vicon(10,:),'b');
title('Angualr Velocity X');
xlabel('Time (s)');
legend('Predicted', 'Actual');
saveas(gcf, [path, dnum, '/Angular Velocity X'], 'png');
%%

plot(sampledTime, yVel,'r', Time, Vicon(8,:),'b');
title('Velocity Y');
legend('Predicted', 'Actual');
saveas(gcf, [path, dnum, '/Velocity Y'], 'png');
plot(sampledTime, yAngVel, 'r',Time, Vicon(11,:),'b');
title('Angualr Velocity Y');
xlabel('Time (s)');
legend('Predicted', 'Actual');
saveas(gcf, [path, dnum, '/Angular Velocity Y'], 'png');

%%

plot(sampledTime, zVel,'r', Time, Vicon(9,:),'b');
title('Velocity Z');
legend('Predicted', 'Actual');
saveas(gcf, [path, dnum, '/Velocity Z'], 'png');
plot(sampledTime, zAngVel, 'r',Time, Vicon(12,:),'b');
title('Angualr Velocity Z');
xlabel('Time (s)');
legend('Predicted', 'Actual');
saveas(gcf, [path, dnum, '/Angular Velocity Z'], 'png');
end



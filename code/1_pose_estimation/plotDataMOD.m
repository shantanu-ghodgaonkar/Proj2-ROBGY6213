function plotData(Tx, Ty, Tz, eulX, eulY, eulZ, data, Vicon, Time, datasetNum)
%PLOTDATA Plot the predicted data
xPos = Tx;
yPos = Ty;
zPos = Tz;
xOrient =eulX;
yOrient = eulY;
zOrient = eulZ;
sampledTime = vertcat(data(:).t);

dnum = ['dataset', int2str(datasetNum)];
path = 'I:/My Drive/Academics/NYUMSMRSPRING24/Robot Localization and Navigation (ROB-GY 6213)/Project/Project2/LaTeX/img/plots/part1/';
figure('Name', sprintf('Model %d - Dataset %d', 1, datasetNum));
%%

plot(sampledTime, xPos,'r', Time, Vicon(1,:),'b');
title('Position X');
legend('Predicted', 'Actual');
saveas(gcf, [path, dnum, '/Position X'], 'png');

plot(sampledTime, xOrient, 'r',Time, Vicon(4,:),'b');
title('Orientation X');
xlabel('Time (s)');
legend('Predicted', 'Actual');
saveas(gcf, [path, dnum, '/Orientation X'], 'png');
%%

plot(sampledTime, yPos,'r', Time, Vicon(2,:),'b');
title('Position Y');legend('Predicted', 'Actual');
saveas(gcf, [path, dnum, '/Position Y'], 'png');

plot(sampledTime, yOrient, 'r',Time, Vicon(5,:),'b');
title('Orientation Y');
xlabel('Time (s)');legend('Predicted', 'Actual');
saveas(gcf, [path, dnum, '/Orientation Y'], 'png');

%%

plot(sampledTime, zPos,'r', Time, Vicon(3,:),'b');
title('Position Z');legend('Predicted', 'Actual');
saveas(gcf, [path, dnum, '/Position Z'], 'png');

plot(sampledTime, zOrient, 'r',Time, Vicon(6,:),'b');
title('Orientation Z');
xlabel('Time (s)');legend('Predicted', 'Actual');
saveas(gcf, [path, dnum, '/Orientation Z'], 'png');
end



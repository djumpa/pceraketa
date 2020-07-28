close all
clearvars

data = readtable('../data/2020-07-28/047.csv');

% Select the range we are interested in
data = data(500:1000,:);

% Barometer data
figure
plot(data.looptime, data.baro_pressure);

figure
plot(data.looptime, data.baro_temp);

% Accelerometer data
figure
ax(1) = subplot(3,1,1);
plot(data.looptime, data.ax);
ax(2) = subplot(3,1,2);
plot(data.looptime, data.ay);
ax(3) = subplot(3,1,3);
plot(data.looptime, data.az);
linkaxes(ax,'x');
linkaxes(ax,'y');

% Gyroscope data
figure
ax(1) = subplot(3,1,1);
plot(data.looptime, data.gx);
ax(2) = subplot(3,1,2);
plot(data.looptime, data.gy);
ax(3) = subplot(3,1,3);
plot(data.looptime, data.gz);
linkaxes(ax,'x');
linkaxes(ax,'y');

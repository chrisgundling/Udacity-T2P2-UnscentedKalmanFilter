%% Unscented Kalman Filter Plotting Tool
% By: Chris Gundling, chrisgundling@gmail.com
clear all;
close all;
clc;

%% Dummy Legend
hold on;
x1 = linspace(390,400,10);
y1 = linspace(390,400,10);
plot(x1,y1,'g.')
plot(x1,y1,'b.')
plot(x1,y1,'k-')
plot(x1,y1,'r-')
legend('Radar','Lidar','GroundTruth','Predictions');

%% Read in the Data
data = readtable('sample-laser-radar-measurement-data-2.txt','Delimiter','tab','ReadVariableNames',false);
ground_truth = readtable('../outputs/output2.txt','Delimiter','tab','ReadVariableNames',false);

%% Plot the Radar and Lidar Measurements
count = 0;
for i = 1 : length(data.Var2)
    if char(data.Var1(i)) == 'R'
        data.Var10(i) = cos(data.Var3(i))*data.Var2(i);
        data.Var11(i) = sin(data.Var3(i))*data.Var2(i);
        plot(data.Var10(i),data.Var11(i),'g.','LineWidth',2)    
        count = count + 1;
    end
    if char(data.Var1(i)) == 'L'
        plot(data.Var2(i),data.Var3(i),'b.','LineWidth',2)
        count = count + 1;
    end
end

%% Plot the grounth truth line
plot(ground_truth.Var8,ground_truth.Var9,'k-','LineWidth',1)

%% Plot the Predicted Path
plot(ground_truth.Var1,ground_truth.Var2,'r-','LineWidth',1)

%% Plot Properties
%axis([4 12 -14 2]);
axis([0 220 -5 40]);
xlabel('px');
ylabel('py');
grid on;

%% RMSE Stats on Output
sq1 = 0;
mse1 = 0;
sq2 = 0;
mse2 = 0;
sq3 = 0;
mse3 = 0;
sq4 = 0;
mse4 = 0;
ground_truth.Var14 = ground_truth.Var3 .* cos(ground_truth.Var4);
ground_truth.Var15 = ground_truth.Var3 .* sin(ground_truth.Var4);


for j = 1:length(ground_truth.Var8)
    sqd1 = ((ground_truth.Var8(j)-ground_truth.Var1(j))^2);
    sq1 = sq1 + sqd1;
    sqd2 = ((ground_truth.Var9(j)-ground_truth.Var2(j))^2);
    sq2 = sq2 + sqd2;
    sqd3 = ((ground_truth.Var10(j)-ground_truth.Var14(j))^2);
    sq3 = sq3 + sqd3;
    sqd4 = ((ground_truth.Var11(j)-ground_truth.Var15(j))^2);
    sq4 = sq4 + sqd4;
end

disp('sum of squares:');
disp(sq1);disp(sq2);disp(sq3);disp(sq4);
mse1 = sq1/length(ground_truth.Var8);
rmse1 = sqrt(mse1);
mse2 = sq2/length(ground_truth.Var8);
rmse2 = sqrt(mse2);
mse3 = sq3/length(ground_truth.Var8);
rmse3 = sqrt(mse3);
mse4 = sq4/length(ground_truth.Var8);
rmse4 = sqrt(mse4);
disp('model evaluated RMSE:'); disp(rmse1),disp(rmse2),disp(rmse3),disp(rmse4);

%% NIS Stats on Output

%% Dummy Legend
figure(2)
hold on;
grid on;
x1 = linspace(390,400,10);
y1 = linspace(390,400,10);
plot(x1,y1,'g.')
plot(x1,y1,'b.')
plot(x1,y1,'k-')
plot(x1,y1,'r-')
legend('NIS-Radar','NIS-Laser','95%-Prob(3d)','95%-Prob(2d)');

%% Determine Radar/Laser NIS
r = 0;
l = 0;
count_r = 0;
count_l = 0;
count_t = 1;

for i = 1 : length(ground_truth.Var2)
    if char(ground_truth.Var13(i)) == 'R'
        ground_truth.Var16(i) = ground_truth.Var12(i);
        count_r = count_r + 1;
        plot(i,ground_truth.Var16(i),'g.','LineWidth',2)
        if ground_truth.Var12(i) < 7.815
            r = r + 1;
        end
        count_t = count_t + 1;
    end
    if char(ground_truth.Var13(i)) == 'L'
        ground_truth.Var16(i) = ground_truth.Var12(i);
        count_l = count_l + 1;
        plot(i,ground_truth.Var16(i),'b.','LineWidth',2) 
        if ground_truth.Var12(i) < 5.99
            l = l + 1;
        end
        count_t = count_t + 1;
    end  
end
for i = 1:length(ground_truth.Var2)
    ground_truth.Var17(i) = 7.815;
end
for i = 1:length(ground_truth.Var2)
    ground_truth.Var18(i) = 5.99;
end
plot(ground_truth.Var17,'k-','LineWidth',2);
plot(ground_truth.Var18,'r-','LineWidth',2);
percent_under_r = 100*r/count_r;
percent_under_l = 100*l/count_l;
xlabel('Example #');
ylabel('NIS Value');
axis([0 200 0 30]);

% Create Annotation
Radar_str = num2str(percent_under_r);
Radar_str = strcat(Radar_str,'%');
Laser_str = num2str(percent_under_l);
Laser_str = strcat(Laser_str,'%');

dim = [.15 .5 .3 .3];
str = strcat('Radar % Under Line:  ',Radar_str);
annotation('textbox',dim,'String',str,'FitBoxToText','on');
dim = [.15 .43 .3 .3];
str = strcat('Laser % Under Line:  ',Laser_str);
annotation('textbox',dim,'String',str,'FitBoxToText','on');

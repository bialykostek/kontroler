clear all
clc
close all
file = importdata("log_27_10_14_40_41.txt");

data = [[0]]
u = 1;
for i=1:length(file)
    tmp = split(file(i), ",");
    if length(tmp) == 34
        if not(str2double(tmp(12)) == 0)
            for k=1:length(tmp)-1 
                data(u, k) = str2double(tmp(k)); 
            end
            u = u + 1;
        end
    end
end


plot(data(:, 16)+150)
hold on
plot(data(:, 12))
legend('Pressure altitude sensor', 'GPS module');
xlabel('Frame number');
ylabel('Relative altitude [m]');
xlim([0 2634])
ylim([0 300])
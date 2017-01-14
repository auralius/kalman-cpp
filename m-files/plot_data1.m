close all;
load ../bin/log_file1.txt;

h = figure;

hold;
plot(log_file1(:,1), log_file1(:,2), 'b','LineWidth',2);
plot(log_file1(:,1), log_file1(:,3), 'r','LineWidth',2);
plot(log_file1(:,1), log_file1(:,4), 'g','LineWidth',2);

legend('measurement', 'true', 'estimate');
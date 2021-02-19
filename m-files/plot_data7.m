close all;
clear all;
load ../bin/log_file7.txt;

% This is the data format:
% iteration# [TAB] measurement [TAB] estimated [EOL]

figure
hold on;
plot(log_file7(:,1), log_file7(:,2), 'b');
plot(log_file7(:,1), log_file7(:,3), '--r', 'LineWidth', 2);
xlabel('# iterations');
ylabel('z');

legend('measurement', 'estimate');
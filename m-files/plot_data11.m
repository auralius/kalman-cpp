close all;
clear all;
load ../bin/log_file11.txt;

% This is the data format:
% iteration# [TAB] true1 [TAB] estimate1 [TAB] 
% true2 [TAB] estimate2 [TAB]
% true3 [TAB] estimate3[TAB] [EOL]

figure;

subplot(3, 1, 1);
hold on;
plot(log_file11(:,1), log_file11(:,2), 'b');
plot(log_file11(:,1), log_file11(:,3), '--r');
ylabel('x_1');


subplot(3, 1, 2);
hold on;
plot(log_file11(:,1), log_file11(:,4), 'b');
plot(log_file11(:,1), log_file11(:,5), '--r');
ylabel('x_2');

subplot(3, 1, 3);
hold on;
plot(log_file11(:,1), log_file11(:,6), 'b');
plot(log_file11(:,1), log_file11(:,7), '--r');
xlabel('n-th iteration');
ylabel('x_3');

legend('true', 'estimate');

figure
hold on;
plot(log_file11(:,1), log_file11(:,8), 'b');
plot(log_file11(:,1), log_file11(:,9), '--r', 'LineWidth', 2);
xlabel('n-th iteration');
ylabel('z');

legend('measurement', 'estimate');
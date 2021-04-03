close all;
clear all;
load ../bin/log_file8.txt;

% This is the data format:
% iteration# [TAB] true1 [TAB] estimate1 [TAB] 
% true2 [TAB] estimate2 [TAB]
% true3 [TAB] estimate3[TAB] [EOL]

figure;

subplot(3, 1, 1);
hold on;
plot(log_file8(:,1), log_file8(:,2), 'b');
plot(log_file8(:,1), log_file8(:,3), '--r', 'LineWidth',1.5);
ylabel('x_1');


subplot(3, 1, 2);
hold on;
plot(log_file8(:,1), log_file8(:,4), 'b');
plot(log_file8(:,1), log_file8(:,5), '--r', 'LineWidth',1.5);
ylabel('x_2');

subplot(3, 1, 3);
hold on;
plot(log_file8(:,1), log_file8(:,6), 'b');
plot(log_file8(:,1), log_file8(:,7), '--r', 'LineWidth',1.5);
xlabel('# iterations');
ylabel('x_3');

legend('true', 'estimate');

figure
hold on;
plot(log_file8(:,1), log_file8(:,8), 'b');
plot(log_file8(:,1), log_file8(:,9), '--r', 'LineWidth',1.5);
xlabel('# iterations');
ylabel('z');

legend('measurement', 'estimate');
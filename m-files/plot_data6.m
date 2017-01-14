close all;
clear all;
load ../bin/log_file6.txt;

% This is the data format:
% iteration# [TAB] true1 [TAB] estimate1 [TAB] 
% true2 [TAB] estimate2 [TAB]
% true3 [TAB] estimate3[TAB] [EOL]

figure;

subplot(3, 1, 1);
hold on;
plot(log_file6(:,1), log_file6(:,2), 'b');
plot(log_file6(:,1), log_file6(:,3), '--r');
ylabel('x_1');


subplot(3, 1, 2);
hold on;
plot(log_file6(:,1), log_file6(:,4), 'b');
plot(log_file6(:,1), log_file6(:,5), '--r');
ylabel('x_2');

subplot(3, 1, 3);
hold on;
plot(log_file6(:,1), log_file6(:,6), 'b');
plot(log_file6(:,1), log_file6(:,7), '--r');
xlabel('# iterations');
ylabel('x_3');

legend('true', 'estimate');

figure
hold on;
plot(log_file6(:,1), log_file6(:,8), 'b');
plot(log_file6(:,1), log_file6(:,9), '--r');
xlabel('# iterations');
ylabel('z');

legend('measurement', 'estimate');
































legend('true', 'estimate');
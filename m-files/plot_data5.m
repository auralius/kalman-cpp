close all;
load ../bin/log_file5.txt;

% This is the data format:
% iteration# [TAB] true1 [TAB] estimate1 [TAB] 
% true2 [TAB] estimate2 [TAB]
% true3 [TAB] estimate3[TAB] [EOL]

figure;

subplot(3, 1, 1);
hold on;
plot(log_file5(:,1), log_file5(:,2), 'b');
plot(log_file5(:,1), log_file5(:,3), '--r');
ylabel('x_1');


subplot(3, 1, 2);
hold on;
plot(log_file5(:,1), log_file5(:,4), 'b');
plot(log_file5(:,1), log_file5(:,5), '--r');
ylabel('x_2');

subplot(3, 1, 3);
hold on;
plot(log_file5(:,1), log_file5(:,6), 'b');
plot(log_file5(:,1), log_file5(:,7), '--r');
xlabel('# iterations');
ylabel('x_3');

legend('true', 'estimate');
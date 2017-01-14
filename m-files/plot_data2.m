close all;
load ../bin/log_file2.txt;

subplot(2,1,1);
hold;
plot(log_file2(:,1), log_file2(:,2), 'b');
plot(log_file2(:,1), log_file2(:,3), 'r');
plot(log_file2(:,1), log_file2(:,6), 'g');
xlabel('position');
legend('true', 'estimate', 'measured');

subplot(2,1,2);
hold
plot(log_file2(:,1), log_file2(:,4), 'b');
plot(log_file2(:,1), log_file2(:,5), 'r');
xlabel('velocity');

legend('true', 'estimate');


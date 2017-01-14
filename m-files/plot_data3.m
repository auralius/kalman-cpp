close all;
load ../bin/log_file3.txt;

figure;

subplot(3,1,1);
hold;
plot(log_file3(:,1), log_file3(:,2), 'b');
plot(log_file3(:,5), log_file3(:,6), 'r');
xlabel('x1');
ylabel('x2');
title('position');
legend('true', 'estimate');

subplot(3,1,2);
hold;
plot(log_file3(:,3), log_file3(:,4), 'b');
plot(log_file3(:,7), log_file3(:,8), 'r');
xlabel('x3');
ylabel('x4');
title('velocity');
legend('true', 'estimate');

subplot(3,1,3);
hold;
plot(log_file3(:,9), log_file3(:,10), 'b');
xlabel('x1');
ylabel('x2');
title('measured position');

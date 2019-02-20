%����ǲ���Parsegov�̶�ʱ��Lyapunov����ʽ�ķ���

clear;
close all;
clc;

delta_time = 0.01;% 0.01 ��ʾÿ�ε���һ�ε�ʱ������0.01���ӣ�Ҳ���ǿ���Ƶ����100hz�����Ƶ����ʵ�����˻��ȿ������ǿ��ж��Ҿ����õ�
total_time_s = 15;% �����ʱ�䳤�ȣ�100��
loop_tick = total_time_s / delta_time;% ѭ�����ܲ���

%ˮ�洬��ģ��
a = 1 / 2;
b = 1 / 4;

%���Ʋ�����ʼ��
k_1 = 1;
k_2 = 1;
alpha_2 = 0.5;
alpha_2 = 2 / 5
alpha_1 = alpha_2 / (2 - alpha_2)
alpha_2_prime = 1.2
alpha_1_prime = alpha_2_prime / (2 - alpha_2_prime)

min_value = 10 / 180 * pi;
max_value = 10;
%*************************************************************��1�ֳ�ʼ���

x1(1) = min_value;
x1(1) = 10 / 180 * pi;
% x1(1) = 1;
% x1(1) = max_value;
x2(1) = 0;

%��������ʼ��
u(1) = 0;

finish_tick = 1;
bool_use_sig = 1;

for i = 1:1:loop_tick
    %x_desired(i) = 2 * sin(i * delta_time);
    x1_desired(i) = 2 * x1(1);
    x1_desired(i) = 0 * x1(1);
    x2_desired(i) = 0;
    
    e1(i) = x1(i) - x1_desired(i);
    e2(i) = x2(i);
    
    if bool_use_sig
        u(i) = -k_1 * (sig(e1(i), alpha_1) + sig(e1(i), 1) + sig(e1(i), alpha_1_prime)) - k_2 * (sig(e2(i), alpha_2) + sig(e2(i), 1) + sig(e2(i), alpha_2_prime));
    else
        u(i) = -k_1 * (atan(sig(e1(i), alpha_1)) + atan(sig(e1(i),1)) + atan(sig(e1(i), alpha_1_prime))) - k_2 * (atan(sig(e2(i), alpha_2)) + atan(sig(e2(i), 1)) + atan(sig(e2(i), alpha_2_prime)));
    end
   
    x1(i + 1) =   x2(i)* delta_time + x1(i);
    x2(i + 1) =  u(i) * delta_time + x2(i);
    x2(i + 1) =  (-a * x2(i) + b * u(i)) * delta_time + x2(i);
    
    finish_tick = i;
end       
time = 1:1:finish_tick;
for i = 1:1:finish_tick
    x11(i) = x1(i);
    x12(i) = x2(i);
    u1(i) = u(i);
end

X = sprintf('desired_tick = %d, finish_tick = %d', loop_tick, finish_tick);
disp(X);

%*************************************************************��1�ֳ�ʼ���
x1(1) = min_value;
x1(1) = 90 / 180 * pi;
% x1(1) = 1;
% x1(1) = max_value;
x2(1) = 0;

%��������ʼ��
u(1) = 0;

finish_tick = 1;

for i = 1:1:loop_tick
    %x_desired(i) = 2 * sin(i * delta_time);
    x1_desired(i) = 2 * x1(1);
    x1_desired(i) = 0 * x1(1);
    x2_desired(i) = 0;
    
    e1(i) = x1(i) - x1_desired(i);
    e2(i) = x2(i);
    
    if bool_use_sig
        u(i) = -k_1 * (sig(e1(i), alpha_1) + sig(e1(i), 1) + sig(e1(i), alpha_1_prime)) - k_2 * (sig(e2(i), alpha_2) + sig(e2(i), 1) + sig(e2(i), alpha_2_prime));
    else
        u(i) = -k_1 * (atan(sig(e1(i), alpha_1)) + atan(sig(e1(i),1)) + atan(sig(e1(i), alpha_1_prime))) - k_2 * (atan(sig(e2(i), alpha_2)) + atan(sig(e2(i), 1)) + atan(sig(e2(i), alpha_2_prime)));
    end
   
    x1(i + 1) =  x2(i)* delta_time + x1(i);
    x2(i + 1) =  u(i) * delta_time + x2(i);
    x2(i + 1) =  (-a * x2(i) + b * u(i)) * delta_time + x2(i);
    
    finish_tick = i;
end       
time = 1:1:finish_tick;
for i = 1:1:finish_tick
    x21(i) = x1(i);
    x22(i) = x2(i);
    u2(i) = u(i);
end

X = sprintf('desired_tick = %d, finish_tick = %d', loop_tick, finish_tick);
disp(X);

%*************************************************************��1�ֳ�ʼ���
x1(1) = min_value;
x1(1) = 150 / 180 * pi;
% x1(1) = 1;
% x1(1) = max_value;
x2(1) = 0;

%��������ʼ��
u(1) = 0;

finish_tick = 1;

for i = 1:1:loop_tick
    %x_desired(i) = 2 * sin(i * delta_time);
    x1_desired(i) = 2 * x1(1);
    x1_desired(i) = 0 * x1(1);
    x2_desired(i) = 0;
    
    e1(i) = x1(i) - x1_desired(i);
    e2(i) = x2(i);
    
    if bool_use_sig
        u(i) = -k_1 * (sig(e1(i), alpha_1) + sig(e1(i), 1) + sig(e1(i), alpha_1_prime)) - k_2 * (sig(e2(i), alpha_2) + sig(e2(i), 1) + sig(e2(i), alpha_2_prime));
    else
        u(i) = -k_1 * (atan(sig(e1(i), alpha_1)) + atan(sig(e1(i),1)) + atan(sig(e1(i), alpha_1_prime))) - k_2 * (atan(sig(e2(i), alpha_2)) + atan(sig(e2(i), 1)) + atan(sig(e2(i), alpha_2_prime)));
    end
   
    x1(i + 1) =   x2(i)* delta_time + x1(i);
    x2(i + 1) =  u(i) * delta_time + x2(i);
    x2(i + 1) =  (-a * x2(i) + b * u(i)) * delta_time + x2(i);
    
    finish_tick = i;
end       
time = 1:1:finish_tick;
for i = 1:1:finish_tick
    x31(i) = x1(i);
    x32(i) = x2(i);
    u3(i) = u(i);
end

X = sprintf('desired_tick = %d, finish_tick = %d', loop_tick, finish_tick);
disp(X);

%*************************************************************��4�ֳ�ʼ���-����ʱ�������
alpha_2 = 2 * alpha_1 / (1 + alpha_1)

x1(1) = min_value;
% x1(1) = 1;
% x1(1) = max_value;
x2(1) = 0;

%��������ʼ��
u(1) = 0;

finish_tick = 1;
bool_use_sig = 1;

for i = 1:1:loop_tick
    %x_desired(i) = 2 * sin(i * delta_time);
    x1_desired(i) = 2 * x1(1);
    x1_desired(i) = 0 * x1(1);
    x2_desired(i) = 0;
    
    e1(i) = x1(i) - x1_desired(i);
    e2(i) = x2(i);
    
    if bool_use_sig
        u(i) = -k_1 * (sig(e1(i), alpha_1) ) - k_2 * (sig(e2(i), alpha_2) );
    else
        u(i) = -k_1 * (atan(sig(e1(i), alpha_1)) ) - k_2 * (atan(sig(e2(i), alpha_2)) );
    end
   
    x1(i + 1) =   x2(i)* delta_time + x1(i);
    x2(i + 1) =  u(i) * delta_time + x2(i);
    x2(i + 1) =  (-a * x2(i) + b * u(i)) * delta_time + x2(i);
    
    finish_tick = i;
end       
time = 1:1:finish_tick;
for i = 1:1:finish_tick
    x41(i) = x1(i);
    x42(i) = x2(i);
    u4(i) = u(i);
end

X = sprintf('desired_tick = %d, finish_tick = %d', loop_tick, finish_tick);
disp(X);


%**************************************************������ͼ
if finish_tick > loop_tick
    finish_tick = loop_tick;
    time = 1:1:finish_tick;
end


figure;
%plot(time * delta_time, u1(1:finish_tick), ':r', time * delta_time, u2(1:finish_tick), '--r', time * delta_time, u3(1:finish_tick), '-.r', time * delta_time, u4(1:finish_tick), 'b');
plot(time * delta_time, u1(1:finish_tick), ':r', time * delta_time, u2(1:finish_tick), '--b', time * delta_time, u3(1:finish_tick), '-.k');
legend('x1(0) = 10^\circ', 'x1(0) = 90^\circ', 'x1(0) = 150^\circ');
xlabel('time(s)');
ylabel('control');

figure;
%plot(time * delta_time, x11(1:finish_tick), ':r', time * delta_time, x21(1:finish_tick), '--r', time * delta_time, x31(1:finish_tick), '-.r', time * delta_time, x41(1:finish_tick), 'b');
plot(time * delta_time, x11(1:finish_tick) * 180 / pi, ':r', time * delta_time, x21(1:finish_tick) * 180 / pi, '--b', time * delta_time, x31(1:finish_tick) * 180 / pi, '-.k');
legend('x1(0) = 10^\circ', 'x1(0) = 90^\circ', 'x1(0) = 150^\circ');
xlabel('time(s)');
ylabel('heading(deg)');
%axis([-inf inf -0.5 8]);

figure;
subplot(2, 1, 1);
plot(time * delta_time, u1(1:finish_tick), ':r', time * delta_time, u4(1:finish_tick), 'b');
legend('Fixed-time control', 'Finite-time control');
xlabel('time(s)');
ylabel('control');
%axis([-inf inf -0.3 0.3]);

hold on;
subplot(2, 1, 2);
plot(time * delta_time, x11(1:finish_tick) * 180 / pi, ':r', time * delta_time, x41(1:finish_tick) *180 / pi, 'b');
legend('Fixed-time control', 'Finite-time control');
ylabel('heading(deg)');










clc
clear all
close all
path = [[1, 3]; [3, 5]; [4, 2]; [2.5, 1.2]; [2, -2.5]];

[Pos, Vel] = minimum_jerk(path);
figure(1)
plot(Pos(2, :), Pos(3, :))
pause(0.01)

x = path(:, 1);
deltaT = 2;
T = linspace(0, deltaT * (length(x) - 1), length(x));

K = 3;
n_order = 2 * K - 1;
M = length(x) - 1;
N = M * (n_order + 1);

Q = zeros(N, N);
for k = 1:M
    Qk = getQk(T(k), T(k + 1));
    Q(6 * (k - 1) + 1:6 * k, 6 * (k - 1) + 1:6 * k) = Qk;
end
Q = 2 * Q;

A0 = zeros(2 * K + M - 1, N);
b0 = zeros(1,size(A0,1));

% 添加首末状态约束(包括位置、速度、加速度)
for k = 0:K-1 % 控制导数
    for i = k:5 % 控制阶数
        c = 1;
        for j = 0:k-1
            c = c * (i - j);
        end
        A0(1 + k*2, i+1) = c * T(1)^(i - k);
        A0(2 + k*2, (M - 1) * 6 +i+1) = c * T(M+1)^(i - k);
    end
end
b0(1) = x(1);
b0(2) = x(M + 1);

% 添加每段轨迹的初始位置约束
for m = 1:M-1
    for i = 0:5
        A0(6 + m, m * 6 + i+1) = T(m + 1)^i;
    end
    b0(6 + m) = x(m + 1);
end

A1 = zeros((M - 1) * 3, N);
b1 = zeros(1,size(A1,1));
for m = 0:M - 2
    for k = 0:2
        for i = 0:5
            c = 1;
            for j = 0:k-1
                c = c * (i - j);
            end
            index = m * 3 + k;
            A1(index+1, m * 6 + i+1) = c * T(m + 2)^(i - k);
            A1(index+1, (m + 1)* 6 + i+1) = -c * T(m + 2)^(i - k);
        end
    end
end

A = [A0; A1];
b = [b0, b1];

result = quadprog(Q, zeros(1,N), zeros(18,24),zeros(1,18),A, b);
p_coff = result;

Pos = [];
Vel = [];
Acc = [];
for k = 0:M-1
    t = linspace(T(k+1), T(k + 2), 100);
    t_pos = [t.^0; t.^1; t.^2; t.^3; t.^4; t.^5];
    t_vel = [t*0; t.^0; 2 * t.^1; 3 * t.^2; 4 * t.^3; 5 * t.^4];
    t_acc = [t*0; t*0; 2 * t.^0; 3 * 2 * t.^1; 4 * 3 * t.^2; 5 * 4 * t.^3];
    coef = p_coff(k*6+1 : (k+1)*6);
    pos = coef' * t_pos;
    vel = coef' * t_vel;
    acc = coef' * t_acc;
    Pos = [Pos, [t; pos]];
    Vel = [Vel, [t; vel]];
    Acc = [Acc, [t; acc]];
end

subplot(3, 1, 1)
plot(Pos(1, :), Pos(2, :))
xlabel("time(s)")
ylabel("position(m)")

subplot(3, 1, 2)
plot(Vel(1, :)', Vel(2, :)')
xlabel("time(s)")
ylabel("velocity(m/s)")

subplot(3, 1, 3)
plot(Acc(1, :)', Acc(2, :)')
xlabel("time(s)")
ylabel("acceleration(m/s^2)")

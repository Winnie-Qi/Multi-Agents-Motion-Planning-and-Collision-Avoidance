function [Pos,Vel] = minimum_jerk(path)

% Define the time interval between points
% deltaT = 2;
% T = linspace(0, deltaT * (length(path) - 1), length(path));

K = 3; % jerk is the 3rd derivative 
n_order = 2 * K - 1; % Polynomial order
M = length(path) - 1; % The number of segments of the trajectory
N = M * (n_order + 1); % The dimension of matrix Q

Pos = []; % Initialize position
Vel = []; % Initialize velocity
distance = sqrt(sum(diff(path).^2, 2)); % Calculate distance between points
relative_times = cumsum(distance / sum(distance)) * 20; 
T = [0;relative_times];
for k = 0:M-1
    t = linspace(T(k+1)+(T(k + 2)-T(k+1))/20, T(k + 2), 20);
    Pos = [Pos, t];
    Vel = [Vel, t];
end
% Pos = [Pos, 20];
% Vel = [Vel, 20];

for d = 1:2
    x = path(:,d); % Extract x or y coordinates
    Q = zeros(N, N); % Initialize Q matrix
    for k = 1:M
        Qk = getQk(T(k), T(k + 1)); % Calculate Qk matrix for each segment
        Q(6 * (k - 1) + 1:6 * k, 6 * (k - 1) + 1:6 * k) = Qk; % Assign Qk to Q matrix
    end
    Q = 2 * Q; % The standard objective function is 1/2xTQx + qTx, so Q need to multiply by 2

    A0 = zeros(2 * K + M - 1, N);
    b0 = zeros(1,size(A0,1));

    % Add the state constraints for the first and last points (including position, velocity, acceleration)
    for k = 0:K-1 
        for i = k:5
            c = 1;
            for j = 0:k-1
                c = c * (i - j);
            end
            A0(1 + k*2, i+1) = c * T(1)^(i - k);
            A0(2 + k*2, (M - 1) * 6 +i+1) = c * T(M+1)^(i - k);
        end
    end
    b0(1) = x(1); % Set initial position constraint
    b0(2) = x(M + 1); % Set final position constraint

    % Add an initial position constraint for each trajectory
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

    p_coff = quadprog(Q, zeros(1,N), zeros(size(A)),zeros(size(b)),A, b);
    
    pos = [];
    vel = [];
    for k = 0:M-1
        t = linspace(T(k+1)+(T(k + 2)-T(k+1))/20, T(k + 2), 20);
        t_pos = [t.^0; t.^1; t.^2; t.^3; t.^4; t.^5];
        t_vel = [t*0; t.^0; 2 * t.^1; 3 * t.^2; 4 * t.^3; 5 * t.^4];
        coef = p_coff(k*6+1 : (k+1)*6);
        p = coef' * t_pos;
        v = coef' * t_vel;
        pos = [pos, p];
        vel = [vel, v];
    end
    Pos = [Pos; pos];
    Vel = [Vel; vel];
end
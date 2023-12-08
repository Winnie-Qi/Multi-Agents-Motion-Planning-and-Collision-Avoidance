function Q = getQk(T_down, T_up)
    Q = zeros(6, 6);
    Q(4, 5) = 72 * (T_up^2 - T_down^2);
    Q(4, 6) = 120 * (T_up^3 - T_down^3);
    Q(5, 6) = 360 * (T_up^4 - T_down^4);
    Q = Q + Q'; % Q is a symmetric matrix
    Q(4, 4) = 36 * (T_up^1 - T_down^1);
    Q(5, 5) = 192 * (T_up^3 - T_down^3);
    Q(6, 6) = 720 * (T_up^5 - T_down^5);
end

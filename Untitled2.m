%% osqp solver
clear,clc,close

% addpath('./src')

start_end_points =  [0,0, 50,3]';
start_end_vel =     [2,0, 10,0]';
a_limit = 3;
T = 10;
plot_flag = 1;

n = 4;

Qsub = [ 1/3     -5/6	1/2     1/6 	-1/6; % n = 4
        -5/6	7/3     -2      1/3     1/6;
        1/2     -2      3       -2      1/2;
        1/6     1/3     -2      7/3     -5/6;
        -1/6    1/6     1/2     -5/6    1/3];

Q = [   Qsub         zeros(5);
        zeros(5)    Qsub];
        
b = zeros(10,1);

A = [   1   zeros(1,9);                 %p00
        zeros(1,5)	1   zeros(1,4);     %p01
        zeros(1,4)  1   zeros(1,5);     %p10
        zeros(1,9)  1;                  %p11
        -1 1 zeros(1,8);                %v00
        zeros(1,5) -1 1 zeros(1,3);     %v01
        zeros(1,3) -1 1 zeros(1,5);     %v10
        zeros(1,8) -1 1                 %v11
        1	-2	1	zeros(1,7);         %a00
        zeros(1,5)  1   -2  1   0   0;  %a01
        0   0	1	-2  1	zeros(1,5); %a10
        zeros(1,7)	1   -2  1;];        %a11

x_warm = [linspace(start_end_points(1),start_end_points(3),n+1)';linspace(start_end_points(2),start_end_points(4),n+1)'];
y_warm = [start_end_points' start_end_vel' ones(1,4)]';

l = [   start_end_points;
        start_end_vel/n*T;
        -a_limit*ones(4,1)/n/(n-1)*T*T];
    
u = [   start_end_points;
        start_end_vel/n*T;
        a_limit*ones(4,1)/n/(n-1)*T*T];

tic
solver = osqp;
solver.setup(Q, b, A, l, u, 'verbose', true);
solver.warm_start('x', x_warm,'y', y_warm);
res = solver.solve();
ControlPoints = reshape(res.x,[],2);
costtime = toc

Index_0to1 = linspace(0,1,101);

rbc = rational_bezier_curve(n,ControlPoints);
[v, v_abs] = calc_vel(n,Index_0to1,T,ControlPoints);
[a, a_abs] = calc_acc(n,Index_0to1,T,ControlPoints);

if plot_flag
    figure
    subplot(5,2,[1 2]),plot(ControlPoints(:,1),ControlPoints(:,2),'*r',rbc(:,1),rbc(:,2),'b'),ylabel('y');xlabel('x');
    subplot(5,2,[3 4]),plot(Index_0to1*T,v_abs),ylabel('|v|');xlabel('t');
    subplot(5,2,5),plot(Index_0to1*T,v(:,1)),ylabel('vx');xlabel('t');
    subplot(5,2,6),plot(Index_0to1*T,v(:,2)),ylabel('vy');xlabel('t');
    subplot(5,2,[7 8]),plot(Index_0to1*T,a_abs),ylabel('|a|');xlabel('t');
    subplot(5,2,9),plot(Index_0to1*T,a(:,1)),ylabel('ax');xlabel('t');
    subplot(5,2,10),plot(Index_0to1*T,a(:,2)),ylabel('ay');xlabel('t');
end
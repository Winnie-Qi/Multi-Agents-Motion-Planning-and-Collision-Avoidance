function new_vel = orca(Pos,Vel,oth_Pos,oth_Vel,deltaT)

r = 0.25; % Radius of each agent

new_vel = 0;
C = [];
V = [];
N = [];
for i = 1:size(oth_Pos)
    % Transform coordinate system into velocity domain
    p_hat_B = (oth_Pos(i,:) - Pos)/deltaT;
    r_hat_B = 2*r/deltaT;
    % if p_hat_B < r_hat_B 如果已经碰撞？
    % else
    v_AB = Vel - oth_Vel(i,:);
    [c,u,n] = FindUN(p_hat_B',r_hat_B,v_AB'); % 一对一    
    N = [N;n'];
    V = [V; Vel + 1/2* u'];
    if c
        C = [C,i]; % record with which plane caused the conflict        
    end
end


if isempty(C) % 如果没有碰撞直接返回原值即可
    [X,Y,VX,VY] = deal(Pos(1),Pos(2),Vel(1),Vel(2));
else
    % Incremental linear programming
    for i = 1:length(C)
        line_point = V(C(i),:);
        line_direction = N(C(i),:);
        oth_point = V([1:C(i)-1, C(i)+1:end],:);
        oth_diretion = N([1:C(i)-1, C(i)+1:end],:);
        [left_dist, right_dist] = line_halfplane_intersect(line_point,line_direction,oth_point,oth_diretion);
        new_vel = point_line_project(line_point,line_direction, Vel, left_dist, right_dist);
        
    end    
end
end


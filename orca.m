function [X,Y,VX,VY] = orca(Pos,Vel,oth_Pos,oth_Vel,deltaT)

r = 0.1; % Radius of each agent


C = 0;
U = [];
for i = 1:size(oth_Pos)
    % Transform coordinate system into velocity domain
    p_hat_B = (oth_Pos(i,:) - Pos')/deltaT;
    r_hat_B = 2*r/deltaT;
    % if p_hat_B < r_hat_B 如果已经碰撞？
    v_AB = Vel' - oth_Vel(i,:);
    c,u,v = FindVO(p_hat_B,r_hat_B,v_AB); % 一对一
    U = [U;u];
    V = [V;v];
    if c
        C = C + 1;        
    end
end

% 增量线性规划

if ~C % 如果没有碰撞直接返回原值即可
    [X,Y,VX,VY] = deal(Pos(1),Pos(2),Vel(1),Vel(2));
end
end


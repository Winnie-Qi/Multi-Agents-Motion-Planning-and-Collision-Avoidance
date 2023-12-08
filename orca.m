function [X,Y,VX,VY] = orca(Pos,Vel,oth_Pos,oth_Vel,deltaT)

r = 0.1; % Radius of each agent


C = 0;
U = [];
for i = 1:size(oth_Pos)
    % Transform coordinate system into velocity domain
    p_hat_B = (oth_Pos(i,:) - Pos')/deltaT;
    r_hat_B = 2*r/deltaT;
    % if p_hat_B < r_hat_B ����Ѿ���ײ��
    v_AB = Vel' - oth_Vel(i,:);
    c,u,v = FindVO(p_hat_B,r_hat_B,v_AB); % һ��һ
    U = [U;u];
    V = [V;v];
    if c
        C = C + 1;        
    end
end

% �������Թ滮

if ~C % ���û����ײֱ�ӷ���ԭֵ����
    [X,Y,VX,VY] = deal(Pos(1),Pos(2),Vel(1),Vel(2));
end
end


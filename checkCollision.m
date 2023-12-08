function C = checkCollision(Pos)

C = struct();
for i = 1:length(Pos) % Add a sub-struct for each agent
    field = sprintf('c%d',i);
    C.(field) = struct();
end

D = 0.3; % Twice the radius of the agent
for t = 1:length(Pos{1})    
    for i = 1:length(Pos)
        for j = i+1:length(Pos)
            if length(Pos{i})>= t && length(Pos{j})>=t
                if norm(Pos{i}(2:3,t)-Pos{j}(2:3,t)) <= D
                    field = sprintf('t%d',t); 
                    if isfield(C.(strcat('c',num2str(i))),field)
                        C.(strcat('c',num2str(i))).(field)=[C.(strcat('c',num2str(i))).(field),j];
                    else
                        C.(strcat('c',num2str(i))).(field)=j;
                    end
                    if isfield(C.(strcat('c',num2str(j))),field)
                        C.(strcat('c',num2str(j))).(field)=[C.(strcat('c',num2str(j))).(field),i];                      
                    else                        
                        C.(strcat('c',num2str(j))).(field)=i;
                    end
                end
            end
        end
    end
end


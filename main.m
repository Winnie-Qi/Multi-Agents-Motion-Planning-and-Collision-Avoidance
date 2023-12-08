% Author: Weijie Qi
% Date: 02/2023

clc
clear all
close all
Manual = 0; % if you don't want to customize the workspace, change the value to 0

figure(1)
axis([0,10,0,10]);
daspect([1 1 1]);
hold on


global obstacles;  
global empty;
global graph;
obstacles = {}; % the number of elements in the Cell equals the number of the obstacles
locations = {}; % saves the start and end locations of all agents
empty = {}; % the number of elements in the Cell equals the numbers of the free decomposed cells

if Manual
    n = input('Please enter the number of obstacles: ');
    % Obtain the number and vertex of obstacles through user input
    for i=1:n
        n_i = input(strcat('Please enter the number of vertices of the no.', num2str(i),' obstacle: '));
        obstacle = zeros(n_i,2);
        for j=1:n_i
            [x,y] = ginput(1);
            hold on;
            plot(x,y,'b.');        
            obstacle(j,:) = [x,y];        
        end
        hold on;
        plot(polyshape(obstacle(:,1),obstacle(:,2)),'FaceColor','b');    
        obstacles =[obstacles;obstacle];    
    end
    % Obtain the starting point and the target point through user input
    n = input('Please enter the number of agents: ');
    for i=1:n
        disp(strcat('Please enter the start point of the no.', num2str(i),' agent: '));
        [x_1,y_1] = ginput(1);
        plot(x_1,y_1,'b.');
        disp(strcat('Please enter the goal point of the no.', num2str(i),' agent: '));
        [x_2,y_2] = ginput(1);
        plot(x_2,y_2,'r.');
        locations =[locations;[x_1,y_1;x_2,y_2]];  
    end
%     disp('Please enter the starting point');
%     [start_x,start_y] = ginput(1);
%     plot(start_x,start_y,'bx');
%     disp('Please enter the direction of the starting point');
%     [direction_x1,direction_y1] = ginput(1); 
%     quiver(start_x,start_y,direction_x1-start_x,direction_y1-start_y,0,'b','LineWidth',1,'MaxHeadSize',1);
%     disp('Please enter the target point');
%     [target_x,target_y] = ginput(1); 
%     plot(target_x,target_y,'rx');
%     disp('Please enter the direction of the target point');
%     [direction_x2,direction_y2] = ginput(1); 
%     quiver(target_x,target_y,direction_x2-target_x,direction_y2-target_y,0,'b','LineWidth',1,'MaxHeadSize',1);
else
    n = 6;    
    obstacles = {[3,8;2.5,7.3;3.5,7.3];[6.25,6.5;6.25,5;7.75,5;7.75,6.5];[2.5,4.5;1.7,3.8;2.1,3;2.9,3;3.3,3.8];...
        [1.5,6.5;1.5,6;2,6;2,6.5];[4.5,4;4.5,2.8;5.5,4];[7.5,2.5;7,1.5;8,1.5]};    
%     [start_x, start_y, direction_x1,direction_y1] = deal(0.75,9.25,1.25,9);
%     [target_x,target_y,direction_x2,direction_y2] = deal(9.2,0.25,9.8,0.25);
    for i = 1:n        
        plot(polyshape(obstacles{i}(:,1),obstacles{i}(:,2)),'FaceColor','b');
        hold on
    end
    locations = {[0.5,9.5;9.5,0.5],[4,1.5;9.5,9.5],[0.5,5;9.5,5]};
    plot([locations{1}(1,1),locations{2}(1,1),locations{3}(1,1)],[locations{1}(1,2),locations{2}(1,2),locations{3}(1,2)],'b.');
    plot([locations{1}(2,1),locations{2}(2,1),locations{3}(2,1)],[locations{1}(2,2),locations{2}(2,2),locations{3}(2,2)],'r.');   
%     plot(start_x,start_y,'bx');
%     quiver(start_x,start_y,direction_x1-start_x,direction_y1-start_y,0,'b','LineWidth',1,'MaxHeadSize',1);
%     plot(target_x,target_y,'rx');
%     quiver(target_x,target_y,direction_x2-target_x,direction_y2-target_y,0,'b','LineWidth',1,'MaxHeadSize',1);
end
pause(0.01)

quadtree([0 0;10 0;10 10;0 10]); % Quadtree cell decomposition
pause(0.01)

graph = graph_generation();
pause(0.01)
% graph:
% the number of elements in the Cell equals the numbers of the free decomposed cells
% in every Cell: 
% 1st column: center point of the this cell
% 2nd column: the row index of the cells connected to this cell
% 3rd column: the distances between this cell and its connected cells

colors = parula(length(locations));
Pos = {};
Vel = {};
for i = 1:length(locations)
    path = Dijkstra(locations{i}(1,1),locations{i}(1,2)); % path: the row index of the last cell that closest to the starting point from the current cell
    color = colors(i, :);
    path_points = find_path(path,locations{i}); % sequence of points that connect from the start point to the target point
    plot(path_points(:, 1), path_points(:, 2),'--','Color', color,'LineWidth',1);
    [pos, vel] = minimum_jerk(path_points);    
    plot(pos(2, :), pos(3, :),'Color', color,'LineWidth',1);
    Pos = [Pos;pos];
    Vel = [Vel;vel];
end

% C=checkCollision(Pos);
for i = 1:length(locations) % Round the time point to a multiple of 0.02
    Pos{i}(1, :) = round(Pos{i}(1, :) / 0.02) * 0.02;    
end

% Save the last updated location and speed of each agent.
cur_Pos = zeros(length(locations),2); 
cur_Vel = zeros(length(locations),2);
for i = 1:length(locations)
    cur_Pos(i,:) = locations{i}(1,:);    
end

% main loop
for t = 0.02:0.02:20
    for i = 1:length(locations)
        if any(abs(Pos{i}(1, :) - t)<0.01)            
            idx = find(abs(Pos{i}(1, :) - t)<0.01);
            oth_Pos = cur_Pos([1:i-1, i+1:end], :);
            oth_Vel = cur_Vel([1:i-1, i+1:end], :);
            deltaT = Pos{i}(1,idx+1) - Pos{i}(1,idx);
            [Pos{i}(2, idx), Pos{i}(3, idx), Vel{i}(2, idx), Vel{i}(3, idx)] = orca(Pos{i}(2:3, idx), Vel{i}(2:3, idx),oth_Pos,oth_Vel,deltaT);
            scatter(Pos{i}(2, idx), Pos{i}(3, idx), [], colors(i, :));
            cur_Pos(i,:) = Pos{i}(2:3, idx); 
            cur_Vel(i,:) = Vel{i}(2:3, idx); 
            pause(0.01)
        else
            continue
        end
    end
end
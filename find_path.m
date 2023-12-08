function  pivot_points = find_path(path,locations)

global graph;
global empty;

[start_x, start_y, target_x, target_y] = deal(locations(1,1),locations(1,2),locations(2,1),locations(2,2));
fastest_connect = [];
pivot_points = [];
len = size(path,1);
for i = 1:len % check which cell contains start point
    t = inpolygon(start_x,start_y,empty{i}(:,1),empty{i}(:,2));
    if t        
        start_cell = i;
        break;
    end
end

for i = 1:len % check which cell contains target point
    t = inpolygon(target_x,target_y,empty{i}(:,1),empty{i}(:,2));
    if t        
        target_cell = i;
        break;
    end
end

i = path(target_cell);
fastest_connect = [target_cell;fastest_connect];
while i ~= start_cell    
    fastest_connect = [i;fastest_connect];
    i = path(i);
end
fastest_connect = [i;fastest_connect];

n = length(fastest_connect);

% save sequence of center points of the fastest connected cells
for i = 1:n
    pivot_points = [pivot_points; [graph{fastest_connect(i),1}(1),graph{fastest_connect(i),1}(2)]];
end

pivot_points(1,:) = [start_x,start_y];
    
pivot_points(n,:) = [target_x,target_y];

% plot
% for i = 1:n-1
%     plot([pivot_points(i,1),pivot_points(i+1,1)],[pivot_points(i,2),pivot_points(i+1,2)],'r','LineWidth',1);  
% end

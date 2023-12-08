function path = Dijkstra(start_x,start_y)
% unseenNodes(changing...,2) The first column: index, the second column:distance
% shortest_distance(len,)
global empty;
global graph;
len = length(empty);
path = zeros(len,1);
unseenNodes = zeros(len,2);
unseenNodes(:,1)=1:len;
unseenNodes(:,2)=inf;
shortest_distance = ones(len,1)*inf;

for i = 1:len % check which cell contains start point
    t = inpolygon(start_x,start_y,empty{i}(:,1),empty{i}(:,2));
    if t
        unseenNodes(i,2) = 0; 
        shortest_distance(i) = 0;        
        break;
    end
end

while ~isempty(unseenNodes)
    [~,I] = min(unseenNodes(:,2)); % sort all the distance in unseen nodes
    i = unseenNodes(I,1); % get the index of the minimun node in graph
    unseenNodes(I,:) = []; % delete this node from unseenNodes
    n_child = size(graph{i,2},1); % get the number of childs of this node
    for c = 1:n_child
        child = graph{i,2}(c);
        new = shortest_distance(i) + graph{i,3}(c);
        if shortest_distance(child) > new
            shortest_distance(child) = new;
            c_i = find(unseenNodes(:,1)==child); 
            unseenNodes(c_i,2) = new;
            path(child) = i;
        end
    end
end
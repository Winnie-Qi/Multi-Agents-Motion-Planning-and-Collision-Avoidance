function graph = graph_generation()
% Function to generate a graph

global empty;
len = length(empty);
graph = cell(len,3);
poly = {};

% Loop through each empty cell
for i = 1:len
    e_cell = empty{i};
    center = [(e_cell(1,1)+e_cell(2,1))/2 (e_cell(1,2)+e_cell(4,2))/2];    
    graph(i,1) = {center};
    % Expand each cell slightly so that adjacent cells have overlapping parts
    e_cell(1,:) = e_cell(1,:)-0.01;
    e_cell(2,1) = e_cell(2,1)+0.01;e_cell(2,2) = e_cell(2,2)-0.01;
    e_cell(3,:) = e_cell(3,:)+0.01;
    e_cell(4,1) = e_cell(4,1)-0.01;e_cell(4,2) = e_cell(4,2)+0.01;
    poly = [poly; {polyshape(e_cell(:,1),e_cell(:,2))}];    
end

% Loop through each cell in the graph and check for intersections
for i = 1:len-1
    plot(graph{i,1}(1),graph{i,1}(2),'.','MarkerSize',8,'MarkerEdgeColor','k');
    j = i+1;    
    while j <= len
        t = intersect(poly{i},poly{j});
        if t.NumRegions            
            graph{i,2} = [graph{i,2};j];                        
            graph{j,2} = [graph{j,2};i]; 
            plot([graph{i,1}(1),graph{j,1}(1)],[graph{i,1}(2),graph{j,1}(2)],'Color',[0.5 0.5 0.5],'LineWidth',0.5);
            d = norm(graph{i,1}-graph{j,1});
            graph{i,3} = [graph{i,3};d];
            graph{j,3} = [graph{j,3};d];
        end
        j = j+1;
    end
end
plot(graph{len,1}(1),graph{len,1}(2),'.','MarkerSize',8,'MarkerEdgeColor','k');
function quadtree(cell)
% 4--------3
% |        |
% |        |
% 1--------2

% g----h----i
% |    |    |
% d----e----f
% |    |    |
% a----b----c

global obstacles; % Access the global variable 'obstacles'
stack = []; % Initialize a stack to store cells for decomposition.
stack = [stack;{cell}]; % Access the global variable 'obstacles'

% Perform quadtree decomposition until the stack is empty.
while ~isempty(stack)    
    % Extract the corner points of the current cell from the top of the stack.
    x1 = stack{end}(1,1); y1 = stack{end}(1,2);
    x2 = stack{end}(2,1); y2 = stack{end}(2,2);
    x3 = stack{end}(3,1); y3 = stack{end}(3,2);
    x4 = stack{end}(4,1); y4 = stack{end}(4,2);
    stack(end)=[]; % Remove the current cell from the stack.
    
    % Calculate the corner points of the four sub-cells.
    a = [x1 y1];
    b = [(x1+x2)/2 y1];
    c = [x2 y2];
    d = [x1 (y2+y3)/2];
    e = [(x1+x2)/2 (y2+y3)/2];
    f = [x2 (y2+y3)/2];
    g = [x4 y4];
    h = [(x1+x2)/2 y4];
    i = [x3 y3];
    
    if y3 - y2 > 1 % Check if the height of the current cell is greater than the threshold.
        % Plot lines to represent the boundaries between the sub-cells.
        plot([h(1),b(1)],[h(2),b(2)],'c');
        plot([d(1),f(1)],[d(2),f(2)],'c');  
        
        % If a sub-cell is occupied, add it to the stack for further decomposition.
        cell_1 = [a;b;e;d];
        occ = cell_checker(cell_1);
        if occ
            stack = [stack;{cell_1}];
        end        
        cell_2 = [b;c;f;e];
        occ = cell_checker(cell_2);
        if occ
            stack = [stack;{cell_2}];
        end
        cell_3 = [e;f;i;h];
        occ = cell_checker(cell_3);
        if occ
            stack = [stack;{cell_3}];
        end
        cell_4 = [d;e;h;g];
        occ = cell_checker(cell_4);
        if occ
            stack = [stack;{cell_4}];
        end
    end
end
function occ = cell_checker(cell)
global obstacles;
global empty;
polycell = polyshape(cell(:,1),cell(:,2));
for i = 1:length(obstacles)
    obstacle = obstacles{i};
        polyobstacle = polyshape(obstacle(:,1),obstacle(:,2));
        t = intersect(polyobstacle,polycell);
        if t.NumRegions
            if inpolygon(cell(:,1),cell(:,2),obstacle(:,1),obstacle(:,2))
                occ = 0;
                return
            end
            occ = 1;              
            return 
        end
            
end
occ = 0;
empty = [empty;{cell}];
% plot(polyshape(cell(:,1),cell(:,2)),'FaceColor','g');
end


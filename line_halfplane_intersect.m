function [left_dist, right_dist] = line_halfplane_intersect(line_point,line_direction,oth_point,oth_diretion)

left_dist = -inf;
right_dist = inf;
for i = 1:size(oth_point,1)
    num = dot(oth_diretion(i,:),line_point - oth_point(i,:));
    den = det([line_direction', oth_diretion']);
    offset = num / den;
    if den > 0
        % Point of intersection is to the right.
        right_dist = min(right_dist, offset);
    else
        % Point of intersection is to the left.
        left_dist = max(left_dist, offset);
    end
end
end


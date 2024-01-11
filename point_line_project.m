function Vel = point_line_project(line_point,line_direction, Vel, left_dist, right_dist)

line_dir = [line_direction(2), -line_direction(1)];
proj_len = dot(Vel - line_point, line_dir);
clamped_len = max(min(proj_len, right_dist), left_dist);
Vel = line_point + line_dir .* clamped_len;
end


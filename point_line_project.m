function Vel = point_line_project(line_point,line_direction, Vel, left_dist, right_dist)

line_dir = [line_direction(1), -line_direction(0)];
proj_len = dot(Vel - line_point, line_dir);
clamped_len = clip(proj_len, left_dist, right_dist);
Vel = line_point + line_dir * clamped_len;
end


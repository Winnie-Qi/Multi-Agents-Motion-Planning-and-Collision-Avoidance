function [c,u,n] = FindUN(p,r,v)

% to decide whether to project onto the disk truncating the VO or onto the sides
dividing_center = p * (1 - r^2 / dot(p,p)); 
if dot(v - dividing_center, dividing_center) < 0 % project onto the disk
    w = v - p;
    n = w/norm(w);
    u = n*r - w; % if v is outside of VO, u is useless but compute it anyway
    c = sqrt(sum((p - v).^2))-r < 0 ; % will collide in the future
else % project onto the line
    leg_len = sqrt(dot(p,p)-r^2); 
    direction_r = sign(det([p, v])) * r;
    rotation = [leg_len,-direction_r;direction_r,leg_len]/norm(p);
    side = rotation * p/norm(p); % already normalized
    n = [side(2);-side(1)];
    if direction_r > 0
        n = -n;
        c = det([v,side]) > 0;
    else
        c = det([v,side]) < 0;
    end
    u = side * dot(v,side) - v;    
end
end


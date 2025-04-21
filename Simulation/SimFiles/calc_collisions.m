function out = calc_collisions(bb, t)
s = size(bb);
p(s(2)) = polyshape();
for i=1:s(2)
    p(i) = polyshape([bb(1,i) bb(2,i);bb(3,i) bb(4,i);bb(5,i) bb(6,i);bb(7,i) bb(8,i)]);
end
out = zeros(s(2),s(2));
vec_len = 15;
for i=1:s(2)
    for j=1:s(2)
        if i==j
            continue;
        end
        temp_p = intersect(p(i),p(j));
        if temp_p.NumRegions > 0
            [temp_x, temp_y] = centroid(temp_p);
            temp_x_2 = temp_x + vec_len * cos(t(i));
            temp_y_2 = temp_y + vec_len * sin(t(i));
            [cx, cy] = centroid(p(j));
            angle = calc_angle([temp_x_2 temp_y_2], [cx cy], [temp_x temp_y]);
            if (angle < pi/2)
                out(i,j) = 1;
            end
        end
    end
end
end

function a = calc_angle(p1, p2, p0)
x0 = p0(1);
y0 = p0(2);
x1 = p1(1);
y1 = p1(2);
x2 = p2(1);
y2 = p2(2);
a = abs(atan2(y2 - y0, x2 - x0) - atan2(y1 - y0, x1 - x0));
end

function d = calc_dist(p1, p2, p0)
x0 = p0(1);
y0 = p0(2);
x1 = p1(1);
y1 = p1(2);
x2 = p2(1);
y2 = p2(2);
d = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1) / sqrt((y2 - y1)^2 + (x2 - x1)^2);
end
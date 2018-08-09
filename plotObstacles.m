function plotObstacles(obst)

if isempty(obst)
    return
end

[m, n] = size(obst);
assert(n == 3);

hold on
for i = 1:m
    th = 0:pi/50:2*pi;
    xunit = obst(i, 3) * cos(th) + obst(i, 1);
    yunit = obst(i, 3) * sin(th) + obst(i, 2);
    plot(xunit, yunit, 'LineWidth', 2, 'Color', [0.1, 0.1, 0.1]);
end

end
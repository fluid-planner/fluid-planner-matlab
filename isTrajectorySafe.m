function safe = isTrajectorySafe(wpts, obst)

if isempty(obst)
    safe = true;
    return;
end

[m, n] = size(obst);
assert(n == 3);

for i = 1:m
    d = sqrt( (wpts(1, :)' - obst(i, 1)).^2 + (wpts(2, :)' - obst(i, 2)).^2 ) - obst(i, 3);
    
    if ~isempty(find(d <= 0))
        safe = false;
        return;
    else
        safe = true;
    end
end
    
end
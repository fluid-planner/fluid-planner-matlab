% a simple implementation of Statistics & Machne Learning Toolbox's
% datasample
function y = datasample(data, k)

idx = randi([1, length(data)], 1, k);
y = data(idx);

end
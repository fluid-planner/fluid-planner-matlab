function x = dubins(v, omega, t, x_prev)
if omega == 0
    x = [v*t*cos(x_prev(3)); v*t*sin(x_prev(3)); t*omega];
else
    x = [v/omega*(sin(x_prev(3)+ t*omega) - sin(x_prev(3))); v/omega*(-cos(x_prev(3)+ t*omega)+cos(x_prev(3))); t*omega];
end
    x = x + x_prev;
end
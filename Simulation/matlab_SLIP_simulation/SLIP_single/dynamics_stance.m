function dx = dynamics_stance(t, x)
    global xi
    global l0
    global k
    global m
    global g

    P       = (k/m)*(l0/(sqrt((xi - x(1))^2 + x(2)^2)) - 1);

    dx      = [x(3);
               x(4);
               P*(x(1) - xi);
               P*x(2) - g];
end

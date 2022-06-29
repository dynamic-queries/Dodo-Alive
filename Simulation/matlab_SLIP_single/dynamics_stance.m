function dx = dynamics_stance(t, x)
    global x_i
    global l0
    global k
    global m
    global g

    P       = (k/m)*(l0/(sqrt((x_i - x(1))^2 + x(2)^2)) - 1);

    dx      = [x(3);
               x(4);
               P*(x(1) - x_i);
               P*x(2) - g];
end

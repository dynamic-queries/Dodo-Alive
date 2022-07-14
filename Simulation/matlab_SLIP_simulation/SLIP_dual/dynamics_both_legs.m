function dx = dynamics_both_legs(t, x)
    global x_i0
    global x_i1
    global l0
    global k
    global m
    global g
    
    P       = (k/m)*(l0/(sqrt((x(1) - x_i0)^2 + x(2)^2)) - 1);
    Q       = (k/m)*(l0/(sqrt((x_i1 - x(1))^2 + x(2)^2)) - 1);

    dx      = [x(3);
               x(4);
               P*(x(1) - x_i0) - Q*(x_i1 - x(1));
               P*x(2) + Q*x(2) - g];
end

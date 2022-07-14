function dx = dynamics_stance_to_leg(t, x)
    global x_i1
    global l0
    global k
    global m
    global g
    
    Q       = (k/m)*(l0/(sqrt((x_i1 - x(1))^2 + x(2)^2)) - 1);

    dx      = [x(3);
               x(4);
               -Q*(x_i1 - x(1));
               Q*x(2) - g];
end

function dx = dynamics_flight(t, x)
    global g

    dx      = [x(3);
               x(4);
               0.0;
               -g];
end

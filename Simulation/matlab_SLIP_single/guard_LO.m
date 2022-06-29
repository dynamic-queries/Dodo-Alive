function [value,isterminal,direction] = guard_LO(t, x)
    global x_i
    global l0
    
    l = sqrt((x(1) - x_i)^2 + x(2)^2);     % l (hind-leg) < l0 until LO

    value      = l0 - l;     %when value==0: terminate
    isterminal = 1;          %termination: true
    direction  = -1;          %locate 0 when function is increasing
end

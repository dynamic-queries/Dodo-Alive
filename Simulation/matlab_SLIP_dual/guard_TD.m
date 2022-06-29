function [value,isterminal,direction] = guard_TD(t, x)
    global alpha0
    global l0
    
    value      = x(2) - l0 * sin(alpha0);   %when value==0: terminate
    isterminal = 1;                         %termination: true
    direction  = -1;                        %locate 0 when function is decreasing
end

function [value,isterminal,direction] = guard_apex(t, x)
    value      = x(4);      %when value==0 (vertical velocity): terminate
    isterminal = 1;         %termination: true
    direction  = -1;        %locate 0 when function is decreasing
end

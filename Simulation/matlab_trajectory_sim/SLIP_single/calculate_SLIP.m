function [x_list, t_list, alpha_phases, xi_values] = calculate_SLIP(x0, params)


%% Initialization

global xi;                          %position of leg
global alpha0;                      
global l0;                          
global k;                           
global m;                           
global g;                           

%--------------------------------------------------------------------------
%Parameters initialization
alpha       = params(1);          %initial attack angle \in [0, +-90]
alpha0      = params(2);          %Angle of attack (desired)
l0          = params(3);          %Resting length
m           = params(4);          %Mass
k           = params(5);          %Stiffness
g           = params(6);          %Gravity
step_width  = params(7);          %Stepwidth
num_steps   = params(8);          %Number of steps
%--------------------------------------------------------------------------

x_detail    = {num_steps*3};            %array of state-vectors in detail
x_phases    = {num_steps*2+1};          %array of state-vectors in phases
x_steps     = {num_steps};              %array of state-vectors in steps
x_full      = [];                       %complete list of state-vectors

t_detail    = {num_steps*3};            %array of time-vectors in detail
t_phases    = {num_steps*2+1};          %array of time-vectors in phases
t_steps     = {num_steps};              %array of time-vectirs in steps
t_full      = [];                       %overall time for simulation

a_phases    = {num_steps*2+1};          %array of attack-angle vectors

tspan       = 0.:step_width:5.0;        %timespan for one phase
xi_vals     = [];                       %list of all leg positions
flight_len  = 0;                        %current duration of last flight
x_phases{1} = [];

opts_flight = odeset('Events', @guard_TD);      %during flight, wait for TD
opts_stance = odeset('Events', @guard_LO);      %during stance, wait for LO
opts_apex   = odeset('Events', @guard_apex);    %after LO, wait for apex


%% Computation
% please look at tutorial 3 for more information and the mechanics used in
% this code structure

for si = 1:num_steps      % indexing steps
    pi = (si-1)*2 + 1;    % indexing phases
    di = (si-1)*3 + 1;    % indexing detailed phases
    
    % flight phase at apex, leg forward by 'alpha0' degrees, wait for TD
    [t,x,te,xe,ie] = ode45(@dynamics_flight,tspan,x0,opts_flight);
    if isempty(xe)
        warning("Aborted calculation at step %d, phase %d, detail %d. Leg underground.", si, pi, di)
        break
    end
    x_detail{di}   = x;
    t_detail{di}   = t;
    x_phases{pi}   = [x_phases{pi}; x];
    t_phases{pi}   = [t_phases{pi}; t];
    xi             = xe(1) + l0 * cos(alpha0);
    xi_vals(si)    = xi;
    flight_len     = flight_len + length(x);
    flight_x       = linspace(-5,5,flight_len).';
    a_phases{pi}   = ((alpha0 - alpha)./(1+exp(-flight_x))) + alpha;
    x0             = xe;
    tspan          = te:step_width:5.0;
    
    % stance phase, wait for leg to lift off (LO)
    [t,x,te,xe,ie] = ode45(@dynamics_stance,tspan,x0,opts_stance);
    if isempty(xe)
        warning("Aborted calculation at step %d, phase %d, detail %d. Mass underground.", si, pi+1, di+1)
        break
    end
    x_detail{di+1} = x;
    t_detail{di+1} = t;
    x_phases{pi+1} = x;
    t_phases{pi+1} = t;
    a_phases{pi+1} = atan2(x(:,2), (xi-x(:,1))) + 2*pi*(x(:,2));
    x0             = xe;
    tspan          = te:step_width:5.0;
    
    % after LO, enter flight phase until apex reached again
    [t,x,te,xe,ie] = ode45(@dynamics_flight,tspan,x0,opts_apex);
    if isempty(xe)
        warning("Aborted calculation at step %d, phase %d, detail %d. Negative attack angle.", si, pi+2, di+2)
        break
    end
    x_detail{di+2} = x;
    t_detail{di+2} = t;
    x_phases{pi+2} = x;
    t_phases{pi+2} = t;
    tspan          = te:step_width:5.0;
    alpha          = atan2(x0(2), (xi-x0(1))) + 2*pi*(x0(2)<0);
    flight_len     = length(x);
    x0             = xe;

    % the following part of code is only for detailed plotting
    x_steps{si}    = [x_detail{di}; x_detail{di+1}; x_detail{di+2}];
    t_steps{si}    = [t_detail{di}; t_detail{di+1}; t_detail{di+2}];
    x_full         = [x_full; x_steps{si}];
    t_full         = [t_full; t_steps{si}];
end

flight_x                   = linspace(-5,5,flight_len).';
a_phases{length(t_phases)} = ((1.57 - alpha)./(1+exp(-flight_x))) + alpha;

x_list = {x_full, x_steps, x_phases, x_detail};   %state-vectors in 4 sizes
t_list = {t_full, t_steps, t_phases, t_detail};   %time-vectors in 4 sizes

alpha_phases = a_phases;
xi_values    = xi_vals;
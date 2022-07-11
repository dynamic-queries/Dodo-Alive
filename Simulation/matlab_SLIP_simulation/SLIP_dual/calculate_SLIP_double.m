function [x_list, t_list, alpha_phases, xi_values] = calculate_SLIP_double(x0, params)


%% Initialization

global x_i0;                            %position of hind-leg
global x_i1;                            %position of fore-leg
global alpha0;                      
global l0;                          
global k;                           
global m;                           
global g;                           

%--------------------------------------------------------------------------
%Parameters initialization
alpha0      = params(1);                %Angle of attack (desired)
l0          = params(2);                %Resting length
m           = params(3);                %Mass
k           = params(4);                %Stiffness
g           = params(5);                %Gravity
step_width  = params(6);                %Stepwidth
num_steps   = params(7);                %Number of steps
%--------------------------------------------------------------------------

x_detail    = {num_steps*3};            %array of state-vectors in detail
x_phases    = {num_steps*2+1};          %array of state-vectors in phases
x_steps     = {num_steps};              %array of state-vectors in steps
x_full      = [];                       %complete list of state-vectors

t_detail    = {num_steps*3};            %array of time-vectors in detail
t_phases    = {num_steps*2+1};          %array of time-vectors in phases
t_steps     = {num_steps};              %array of time-vectirs in steps
t_full      = [];                       %overall time for simulation

tspan       = 0.:step_width:5.0;        %timespan for one phase
x_i0        = x0(1);                    %initial position of grounded leg
xi_vals     = x_i0;                     %list of all leg positions
x_phases{1} = [];                       %initialization needed for syntax

opts_one_leg   = odeset('Events', @guard_TD_foreleg);  %during one-leg, wait for TD
opts_both_legs = odeset('Events', @guard_LO_hindleg);  %during both-legs, wait for LO
opts_apex      = odeset('Events', @guard_apex_leg);    %after LO, wait for apex on one leg


%% Computation
% please look at the 2006 paper: "Geyer et al. - Compliant leg behaviour 
% explains basic dynamics of walking and running" for a detailed 
% explanation on the dual SLIP model and the mechanics used in this code 
% structure

for si = 1:num_steps      % indexing steps
    pi = (si-1)*2 + 1;    % indexing phases
    di = (si-1)*3 + 1;    % indexing detailed phases

    % hind-leg at apex, fore-leg forward by 'alpha0' degrees, wait for TD
    [t,x,te,xe,ie] = ode45(@dynamics_leg_to_stance,tspan,x0,opts_one_leg);
    if isempty(xe)
        warning("Aborted calculation at step %d, phase %d, detail %d. Fore-Leg underground.", si, pi, di)
        break
    end
    x_detail{di}   = x;
    t_detail{di}   = t;
    x_phases{pi}   = [x_phases{pi}; x];
    t_phases{pi}   = [t_phases{pi}; t];
    x_i1           = xe(1) + l0 * cos(alpha0);
    xi_vals        = [xi_vals; x_i1];
    x0             = xe;
    tspan          = te:step_width:5.0;
    
    % stance on both legs, wait for hind-leg to lift off (LO)
    [t,x,te,xe,ie] = ode45(@dynamics_both_legs,tspan,x0,opts_both_legs);
    if isempty(xe)
        warning("Aborted calculation at step %d, phase %d, detail %d. Mass underground.", si, pi+1, di+1)
        break
    end
    x_detail{di+1} = x;
    t_detail{di+1} = t;
    x_phases{pi+1} = x;
    t_phases{pi+1} = t;
    x_i0           = x_i1;
    x0             = xe;
    tspan          = te:step_width:5.0;
    
    % after LO, enter one-leg-stance phase until fore-leg apex reached
    [t,x,te,xe,ie] = ode45(@dynamics_stance_to_leg,tspan,x0,opts_apex);
    if isempty(xe)
        warning("Aborted calculation at step %d, phase %d, detail %d. Dodo fell over :(.", si, pi+2, di+2)
        break
    end
    x_detail{di+2} = x;
    t_detail{di+2} = t;
    x_phases{pi+2} = x;
    t_phases{pi+2} = t;
    x0             = xe;
    tspan          = te:step_width:5.0;

    % the following part of code is only for detailed plotting
    x_steps{si}    = [x_detail{di}; x_detail{di+1}; x_detail{di+2}];
    t_steps{si}    = [t_detail{di}; t_detail{di+1}; t_detail{di+2}];
    x_full         = [x_full; x_steps{si}];
    t_full         = [t_full; t_steps{si}];
end

x_list = {x_full, x_steps, x_phases, x_detail};   %state-vectors in 4 sizes
t_list = {t_full, t_steps, t_phases, t_detail};   %time-vectors in 4 sizes

alpha_phases = {};                 %empty because animation not implemented
xi_values    = xi_vals;


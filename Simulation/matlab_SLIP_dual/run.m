clc; clear all;


%% Initialization

global x_i0;                            %position of hind-leg
global x_i1;                            %position of fore-leg
global alpha0;                      
global l0;                          
global k;                           
global m;                           
global g;                           

%--------------------------------------------------------------------------
%Change parameters ONLY here!!
x0          = [0.; 0.98; 1.14; 0.];     %Initial state x = [x y dx dy]^T 
step_width  = 0.001;                    %Stepwidth for ODE solving
num_steps   = 3;                        %Number of steps
alpha0      = 1.2;                      %Angle of attack
l0          = 1.0;                      %Resting length
m           = 80.0;                     %Mass
k           = 15696.0;                  %Stiffness
g           = 9.81;                     %Gravity
%--------------------------------------------------------------------------

x_phases    = {num_steps*3};            %array of state-vectors in phases
t_phases    = {num_steps*3};            %array of time-vectors in phases
x_steps     = {num_steps};              %array of state-vectors in steps
t_steps     = {num_steps};              %array of time-vectirs in steps
x_full      = [];                       %complete list of state-vectors
t_full      = 0;                        %overall time for simulation
tspan       = 0.:step_width:5.0;        %timespan for one phase
x_i0        = x0(1);                    %initial position of grounded leg

opts_one_leg   = odeset('Events', @guard_TD);      %during one-leg, wait for TD
opts_both_legs = odeset('Events', @guard_LO);      %during both-legs, wait for LO
opts_apex      = odeset('Events', @guard_apex);    %after LO, wait for apex on one leg


%% Computation
% please look at the 2006 paper: "Geyer et al. - Compliant leg behaviour 
% explains basic dynamics of walking and running" for a detailed 
% explanation on the dual SLIP model and the mechanics used in this code 
% structure

for i = 1:num_steps     % indexing steps
    j = (i-1)*3 + 1;    % indexing phases

    % hind-leg at apex, fore-leg forward by 'alpha0' degrees, wait for TD
    [t,x,te,xe,ie] = ode45(@dynamics_leg_to_stance,tspan,x0,opts_one_leg);
    x_phases{j}    = x;
    t_phases{j}    = t(end);
    x_i1           = xe(1) + l0 * cos(alpha0);
    x0             = xe;
    tspan          = te:step_width:5.0;
    
    % stance on both legs, wait for hind-leg to lift off (LO)
    [t,x,te,xe,ie] = ode45(@dynamics_both_legs,tspan,x0,opts_both_legs);
    x_phases{j+1}  = x;
    t_phases{j+1}  = t(end);
    x_i0           = x_i1;
    x0             = xe;
    tspan          = te:step_width:5.0;
    
    % after LO, enter one-leg-stance phase until fore-leg apex reached
    [t,x,te,xe,ie] = ode45(@dynamics_stance_to_leg,tspan,x0,opts_apex);
    x_phases{j+2}  = x;
    t_phases{j+2}  = t(end);
    x0             = xe;
    tspan          = te:step_width:5.0;

    % the following part of code is only for detailed plotting
    x_steps{i}     = [x_phases{j}; x_phases{j+1}; x_phases{j+2}];
    t_steps{i}     = sum([t_phases{j:(j+2)}]);
    x_full         = [x_full; x_steps{i}];
    t_full         = sum([t_full; t_steps{i}]);
end



%% Plotting
x_list = {x_full, x_steps, x_phases};       %state-vectors in 3 sizes
t_list = {t_full, t_steps, t_phases};       %time-vectors in 3 sizes

% You can change the type of plot by specifying the format as the 1st 
% parameter in the function "plotting" (The time used for the different 
% parts is printed automatically):
% + 'full'  : plot the full trajectory in one color
% + 'steps' : plot all steps in the trajectory as a different color
% + 'phases': plot reoccuring phases in the trajectory as the same color
plotting('phases', x_list, t_list)

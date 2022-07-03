clc; clear all;


%% Initialization

global x_i;                         %position of leg
global alpha0;                      
global l0;                          
global k;                           
global m;                           
global g;                           

%--------------------------------------------------------------------------
%Change parameters ONLY here!!
x0          = [0.; 1.; 5.; 0.];     %initial state x = [x y dx dy]^T 
step_width  = 0.001;                %Stepwidth
num_steps   = 10;                   %Number of steps
alpha0      = deg2rad(68);          %Angle of attack
l0          = 1.0;                  %Resting length
m           = 80.0;                 %Mass
k           = 20000.0;              %Stiffness
g           = 9.81;                 %Gravity
%--------------------------------------------------------------------------

x_phases    = {num_steps*3};            %array of state-vectors in phases
t_phases    = {num_steps*3};            %array of time-vectors in phases
x_steps     = {num_steps};              %array of state-vectors in steps
t_steps     = {num_steps};              %array of time-vectirs in steps
x_full      = [];                       %complete list of state-vectors
t_full      = [];                       %overall time for simulation
tspan       = 0.:step_width:5.0;        %timespan for one phase

opts_flight = odeset('Events', @guard_TD);      %during flight, wait for TD
opts_stance = odeset('Events', @guard_LO);      %during stance, wait for LO
opts_apex   = odeset('Events', @guard_apex);    %after LO, wait for apex


%% Computation
% please look at tutorial 3 for more information and the mechanics used in
% this code structure

for i = 1:num_steps     % indexing steps
    j = (i-1)*3 + 1;    % indexing phases

    % flight phase at apex, leg forward by 'alpha0' degrees, wait for TD
    [t,x,te,xe,ie] = ode45(@dynamics_flight,tspan,x0,opts_flight);
    x_phases{j}    = x;
    t_phases{j}    = t;
    x_i            = xe(1) + l0 * cos(alpha0);
    x0             = xe;
    tspan          = te:step_width:5.0;
    
    % stance phase, wait for leg to lift off (LO)
    [t,x,te,xe,ie] = ode45(@dynamics_stance,tspan,x0,opts_stance);
    x_phases{j+1}  = x;
    t_phases{j+1}  = t;
    x0             = xe;
    tspan          = te:step_width:5.0;
    
    % after LO, enter flight phase until apex reached again
    [t,x,te,xe,ie] = ode45(@dynamics_flight,tspan,x0,opts_apex);
    x_phases{j+2}  = x;
    t_phases{j+2}  = t;
    x0             = xe;
    tspan          = te:step_width:5.0;

    % the following part of code is only for detailed plotting
    x_steps{i}     = [x_phases{j}; x_phases{j+1}; x_phases{j+2}];
    t_steps{i}     = [t_phases{j}; t_phases{j+1}; t_phases{j+2}];
    x_full         = [x_full; x_steps{i}];
    t_full         = [t_full; t_steps{i}];
end


%% Plotting
x_list = {x_full, x_steps, x_phases};       %state-vectors in 3 sizes
t_list = {t_full, t_steps, t_phases};       %time-vectors in 3 sizes

% The time used for the different parts is printed automatically.
% You can plot the trajectory over time instead of x(1), just turn the 2nd
% parameter of the function to "true".
% You can change the type of plot by specifying the format as the 1st 
% parameter in the function:
% + 'full'  : plot the full trajectory in one color
% + 'steps' : plot all steps in the trajectory as a different color
% + 'phases': plot reoccuring phases in the trajectory as the same color
plotting('phases', true, x_list, t_list)

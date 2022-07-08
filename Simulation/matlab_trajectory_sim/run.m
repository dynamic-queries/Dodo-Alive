clc; clear all;


%% Initialization

%--------------------------------------------------------------------------
%Change parameters ONLY here!!
x0          = [0.; 1.0; 5.0; 0.];   %initial state x = [x y dx dy]^T 
alpha       = deg2rad(90);          %initial attack angle \in [0, +-90]
alpha0      = deg2rad(68);          %Angle of attack (desired)
l0          = 1.0;                  %Resting length
m           = 80.0;                 %Mass
k           = 20000.0;              %Stiffness
g           = 9.81;                 %Gravity

step_width  = 0.001;                %Stepwidth
num_steps   = 10;                   %Number of steps

anim_freq   = 0.0005;               %Frequency of animation updates
load_ws     = true;                 %load workspace or calculate anew
%--------------------------------------------------------------------------


%% Calculation of SLIP single CoM-trajectory and alpha-trajectory

% x_list = {"complete CoM-tr", 
%           "CoM-tr divided in steps (from apex to apex)", 
%           "CoM-tr divided in phases (flight & stance phases)",
%           "CoM-tr divided in phase-details (fl.->apex, st., apex->fl.)"}
% t_list = {"complete time for CoM-tr",
%           "time for CoM-tr steps",
%           "time for CoM-tr phases",
%           "time for CoM-tr phase-details"}
% a_phases := "attack angle over time, divided in phases"
% xi_vals  := "list of all contact points leg to ground"

if (load_ws && isfile('ws_SLIP.mat'))
    load ws_SLIP x_list t_list a_phases xi_vals
else
    params = [alpha, alpha0, l0, m, k , g, step_width, num_steps];
    [x_list, t_list, a_phases, xi_vals] = calculate_SLIP(x0, params);
    save ws_SLIP x_list t_list a_phases xi_vals
end


%% Joint trajectories

%toe_phases = func_toe_trajectory(x_phases, a_phases, xi_vals);


%% Plotting

% The time used for the different parts is printed automatically.
% You can plot the trajectory over time instead of x(1), just turn the 2nd
% parameter of the function to "true".
% You can change the type of plot by specifying the format as the 1st 
% parameter in the function:
% + 'full'   : plot the full trajectory in one color
% + 'steps'  : plot all steps in the trajectory as a different color
% + 'phases' : plot all phases in the trajectory as the same color
% + 'detailed: plot all step-phases in the trajectory as the same color
hold on
func_plotting('phases', false, x_list, t_list)
hold off


%% Animation

% Uncomment this line to open a window with an animation of the leg
% following the plotted trajectory.
%func_animation(x_list{3}, a_phases, anim_freq, xi_vals, l0)
clc; clear all;


%% Initialization

x_full = [];                        %final x-array
t_full = [];                        %final time-array
global x_i0;                        %position of hind-leg
global x_i1;                        %position of fore-leg
global alpha0;                      
global l0;                          
global k;                           
global m;                           
global g;                           

%--------------------------------------------------------------------------
%Change parameters ONLY here!!
x0          = [0.; 1.; 5.; 0.];     %Initial state x = [x y dx dy]^T 
step_width  = 0.001;                %Stepwidth for ODE solving
alpha0      = deg2rad(68);          %Angle of attack
l0          = 1.0;                  %Resting length
m           = 80.0;                 %Mass
k           = 20000.0;              %Stiffness
g           = 9.81;                 %Gravity
%--------------------------------------------------------------------------

tspan       = 0.:step_width:5.0;    %timespan for one phase
x_i0        = x0(1);                %initial position of grounded leg

opts_one_leg   = odeset('Events', @guard_TD);      %during one-leg, wait for TD
opts_both_legs = odeset('Events', @guard_LO);      %during both-legs, wait for LO
opts_apex      = odeset('Events', @guard_apex);    %after LO, wait for apex on one leg


%% Computation
% please look at the 2006 paper: "Geyer et al. - Compliant leg behaviour 
% explains basic dynamics of walking and running" for a detailed 
% explanation on the dual SLIP model and the mechanics used in this code 
% structure

% left leg at apex, right leg forward by 'alpha0' degrees, wait for TD
[t,x1,te,xe,ie] = ode45(@dynamics_leg_to_stance,tspan,x0,opts_one_leg);
x_full          = [x_full; x1];
t_full          = [t_full; t];
x_i1            = xe(1) + l0 * cos(alpha0);

% stance on both legs, wait for left leg to lift off (LO)
x0              = xe;
tspan           = te:step_width:5.0;
[t,x2,te,xe,ie] = ode45(@dynamics_both_legs,tspan,x0,opts_both_legs);
x_full          = [x_full; x2];
t_full          = [t_full; t];
x_i0            = x_i1;

% after LO, enter one-leg-stance phase until right leg apex reached
x0              = xe;
tspan           = te:step_width:5.0;
[t,x3,te,xe,ie] = ode45(@dynamics_stance_to_leg,tspan,x0,opts_apex);
x_full          = [x_full; x3];
t_full          = [t_full; t];


%% Plotting
% final plot (currently commented out for debugging)
% plot(x_full(:,1), x_full(:,2))     

% plot for debugging
plot(x1(:,1), x1(:,2), 'LineWidth', 2)
hold on
plot(x2(:,1), x2(:,2), 'LineWidth', 2)
plot(x3(:,1), x3(:,2), 'LineWidth', 2)
hold off
legend('left leg','both legs', 'right leg')
grid

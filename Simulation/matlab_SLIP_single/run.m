clc; clear all;


%% Initialization

global x_i;                         %position of leg
x_full = [];                        %final x-array
t_full = [];                        %final time-array
global alpha0;                      
global l0;                          
global k;                           
global m;                           
global g;                           

%--------------------------------------------------------------------------
%Change parameters ONLY here!!
x0          = [0.; 1.; 5.; 0.];     %initial state x = [x y dx dy]^T 
step_width  = 0.001;                %Stepwidth
alpha0      = deg2rad(68);          %Angle of attack
l0          = 1.0;                  %Resting length
m           = 80.0;                 %Mass
k           = 20000.0;              %Stiffness
g           = 9.81;                 %Gravity
%--------------------------------------------------------------------------

tspan       = 0.:step_width:5.0;    %timespan for one phase
x_i0        = x0(1);                %initial position of grounded leg

opts_flight = odeset('Events', @guard_TD);      %during one-leg, wait for TD
opts_stance = odeset('Events', @guard_LO);      %during both-legs, wait for LO
opts_apex   = odeset('Events', @guard_apex);    %after LO, wait for apex on one leg


%% Computation
% please look at tutorial 3 for more information and the mechanics used in
% this code structure

% flight phase at apex, leg forward by 'alpha0' degrees, wait for TD
[t,x1,te,xe,ie] = ode45(@dynamics_flight,tspan,x0,opts_flight);
x_full          = [x_full; x1];
t_full          = [t_full; t];
x_i             = xe(1) + l0 * cos(alpha0);

% stance phase, wait for leg to lift off (LO)
x0              = xe;
tspan           = te:step_width:5.0;
[t,x2,te,xe,ie] = ode45(@dynamics_stance,tspan,x0,opts_stance);
x_full          = [x_full; x2];
t_full          = [t_full; t]; 

% after LO, enter flight phase until apex reached again
x0              = xe;
tspan           = te:step_width:5.0;
[t,x3,te,xe,ie] = ode45(@dynamics_flight,tspan,x0,opts_apex);
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
legend('apex-flight','stance', 'flight-apex')
grid

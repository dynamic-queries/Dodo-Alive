clc; clear all;


%% Initialization

%--------------------------------------------------------------------------
%Change parameters ONLY here!! (and for testcase generation also in SLIP)

num_joints_links = 3;                   %number of joints and linkages (2 or 3)
test_case        = 'vdrop';             %run test case (dpend/vdrop/gjump)
show_tau         = true;                %display tau during animation

length_links     = [0.18, 0.18, 0.18];  %length of linkages
mass_links       = [1.0, 1.0, 1.0];     %masses of linkages
com_links        = [0.09, 0.09, 0.09];  %CoM of linkage i in x-dir of frame i
spring_stiffness = 20000;               %Stiffness of springs in pulley-spring
radius_pulley    = 0.0015;              %Radius of pulleys in pulley-spring
q2_resting       = deg2rad(270);        %Angle of q2 so that spring is resting
init_angle       = [deg2rad(296.5664); 
                    deg2rad(269.9930)]; %initial angle of joints
mass_toe         = 0.6;                 %mass of the toe
mass_body        = 9.66;                %mass of the body
sim_time         = 10.0;                %simulation time of test cases
time_step        = 0.1;                 %width of one time-step in simulation

%--------------------------------------------------------------------------

% assertions for checking if input parameters are logically correct
assert((num_joints_links == 2) || (num_joints_links == 3))
assert(length(length_links) == num_joints_links)
assert(length(mass_links) == num_joints_links)
assert(length(com_links) == num_joints_links)
assert(all(com_links <= length_links))
assert(all(size(init_angle)==[2,1]))

assert((init_angle(1) > pi) || (init_angle(1) < 2*pi))
if num_joints_links == 2
    assert((init_angle(2) > 0.0) || (init_angle(2) < pi))
else
    assert((init_angle(2) > pi) || (init_angle(2) < 2*pi))
end


%% Setup
% Set up our model with kinematics & dynamics for the given number of
% joints who all move around the same z-axis.
% Postion vectors p will include 3 entries:
%   x coordinate
%   y coordinate
%   orientation
%   p = ["x position", "y position", "orientation"].


% Get FORWARD KINEMATICS of our model.
% RETURN-TYPE: cell containing function handles.
% p_0i are position-vectors from base-frame to frame {i} and are
% represented in code as function handles (vectors with variable inputs).
% They are stored within a cell to represent FK to different joints.
%   forward_kinematics 
%   = {p_01,p_02,p_03}      [2 joints] -> inputs {q1, q1q2, q1q2}
%   = {p_01,p_02,p_03,p_04} [3 joints] -> inputs {q1, q1q2, q1q2, q1q2}

forward_kinematics = get_forward(num_joints_links, ...
                                 length_links);


% Get INVERSE KINEMATICS of our model.
% RETURN-TYPE: function handle
%   inverse_kinematics 
%   = q [2 joints] -> inputs (xy-position of toe relative to the hip,
%                             previous joint configuration)
%   = q [3 joints] -> inputs (xy-position & orientation of toe rel. to hip,
%                             previous joint configuration)

inverse_kinematics = get_inverse(num_joints_links, ...
                                 length_links, ...
                                 forward_kinematics);


% Get DYNAMICS of our model.
% RETURN-TYPE: cell containing function handles.
% Get the torque vector of all joints as function handles, which take a
% q-vector, qdot-vector and qdotdot-vector as input.
%   dynamics
%   = {tau1(q1,q2,q1d,q2d,q1dd,q2dd),
%      tau2(q1,q2,q1d,q2d,q1dd,q2dd)}               [2 joints]
%   = {tau1(q1,q2,q3,q1d,q2d,q3d,q1dd,q2dd,q3dd),
%      tau2(q1,q2,q3,q1d,q2d,q3d,q1dd,q2dd,q3dd)}   [3 joints]

dynamics           = get_dynamics(num_joints_links, ...
                                  length_links, ...
                                  mass_links, ...
                                  com_links, ...
                                  mass_toe, ...
                                  spring_stiffness, ...
                                  q2_resting, ...
                                  radius_pulley);



%% Trajectories
% Different test cases generate different trajectories. Trajectories in our
% case have to be calculated "by hand" with some logic. The SLIP model
% simulations can help coming up with trajectories for a given test case.
% The get-functions take the desired simulation time (in seconds) as input 
% parameter.
% Test cases:
% + double-pendulum hanging from the ceiling
% + vertical drop
% + gait jumping (not implemented)

% returns a cell containing:
% [dpend] -> {time vector,
%             q1 vector, q1_dot vector, q1_dotdot vector,
%             q2 vector, q2_dot vector, q2_dotdot vector}
% [vdrop] -> {time vector,
%             base-frame (hip) trajectory vector in y-direction,
%             q1 vector, q1_dot vector, q1_dotdot vector,
%             q2 vector, q2_dot vector, q2_dotdot vector,
%             q3 vector, q3_dot vector, q3_dotdot vector}
joint_trajectories = specify_testcase(test_case, num_joints_links, ...
                                      sim_time, time_step, ...
                                      length_links, ...
                                      mass_links, ...
                                      com_links, ...
                                      mass_toe, ...
                                      mass_body, ...
                                      init_angle, ...
                                      inverse_kinematics);



%% Torque profiles
% Get the torque profiles for the specified testcase.

torque_profiles = get_torques(test_case, joint_trajectories, dynamics);



%% Visualize
% Visualize test cases.

% Plot torque profiles over time.
plot_torque_profiles(joint_trajectories{1}, torque_profiles);

% Animate specified test case.
animate(test_case, ...
        show_tau, ...
        joint_trajectories, ...
        torque_profiles, ...
        length_links, ...
        forward_kinematics)

clc; clear all;

% current TODO:
% + add dynamics for 3 links
% + add K(t) to dynamics
% + implement get_torques_vdrop()
% + fix animate_vdrop()



%% Initialization

%--------------------------------------------------------------------------
%Change parameters ONLY here!! (and for testcase generation also in SLIP)

num_joints_links = 3;                   %number of joints and linkages (2 or 3)
length_links     = [0.18, 0.18, 0.18];  %length of linkages
mass_links       = [1.0, 1.0, 1.0];     %masses of linkages
com_links        = [0.09, 0.09, 0.09];  %CoM of linkage i in x-dir of frame i
spring_stiffness = 100;                 %Stiffness of the spring
q2_resting       = deg2rad(270);        %Angle of q2 so that spring is resting
init_angle       = [deg2rad(315); 
                    deg2rad(270)];      %initial angle of joints
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
% Get the torque vector of all joints as function handles, which take a
% q-vector, qdot-vector and qdotdot-vector as input.
% ONLY IMPLEMENTED FOR A 2-LINK SYSTEM
% returns a cell containing:
%   {tau1(q1,q2,q1d,q2d,q1dd,q2dd),
%    tau2(q1,q2,q1d,q2d,q1dd,q2dd)}

dynamics           = get_dynamics(num_joints_links, ...
                                  length_links, ...
                                  mass_links, ...
                                  com_links, ...
                                  mass_toe, ...
                                  spring_stiffness, ...
                                  q2_resting);



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
%   {time vector,
%    q1 vector, q1_dot vector, q1_dotdot vector,
%    q2 vector, q2_dot vector, q2_dotdot vector}
if isfile("trajectory_dpendulum.mat")
    load trajectory_dpendulum.mat jtrajectories_dpendulum
else
    jtrajectories_dpendulum = testcase_double_pendulum(sim_time, ...
                                                       time_step);
    save trajectory_dpendulum.mat jtrajectories_dpendulum
end

% returns a cell containing:
%   {time vector,
%    base-frame (hip) trajectory vector,
%    q1 vector, q1_dot vector, q1_dotdot vector,
%    q2 vector, q2_dot vector, q2_dotdot vector,
%    q3 vector, q3_dot vector, q3_dotdot vector}
if isfile("trajectory_vdrop.mat")
    load trajectory_vdrop.mat jtrajectories_vdrop
else
    jtrajectories_vdrop = testcase_vertical_drop(length_links, ...
                                                 mass_links, ...
                                                 com_links, ...
                                                 mass_toe, ...
                                                 mass_body, ...
                                                 init_angle, ...
                                                 inverse_kinematics, ...
                                                 forward_kinematics);
    save trajectory_vdrop.mat jtrajectories_vdrop
end

% NOT IMPLEMENTED
%jtrajectories_gjump = testcase_gait_jumping(sim_time, time_step);



%% Torque profiles
% Get the torque profiles for the specified trajectories and compare them
% to the specifications of the Open Dynamics motors.

%torque_profiles_dpend = get_torques_dpend(jtrajectories_dpendulum, dynamics);
torque_profiles_vdrop = get_torques_vdrop(jtrajectories_vdrop, dynamics);



%% Visualize
% Visualize test cases.

%plotting(jtrajectories_dpendulum{1}, torque_profiles);
%animate_dpend(jtrajectories_dpendulum, torque_profiles_dpend, length_links, forward_kinematics)
animate_vdrop(jtrajectories_vdrop, [], forward_kinematics)
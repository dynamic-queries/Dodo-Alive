clc; clear all;

% current TODO:
% + add vertical jump testcase



%% Initialization

%--------------------------------------------------------------------------
%Change parameters ONLY here!!

num_joints_links = 2;               %number of joints and linkages (2 or 3)
length_links     = [0.2, 0.2];      %length of linkages
mass_links       = [0.18, 0.18];    %masses of linkages
com_links        = [0.1, 0.1];      %CoM of linkage i in x-dir of frame i
spring_stiffness = 100;             %Stiffness of the spring
q2_resting       = deg2rad(90);     %Angle of q2 so that spring is resting
init_angle       = [deg2rad(270); 
                    deg2rad(90)];   %initial angle of joints
init_pos_hip     = [0.0; 
                    2.0;
                    init_angle(1)]; %initial position of base-frame
mass_toe         = 0.001;           %mass of the toe
sim_time         = 10.0;            %simulation time of test cases
time_step        = 0.1;             %width of one time-step in simulation

%--------------------------------------------------------------------------

% assertions for checking if input parameters are logically correct
assert((num_joints_links == 2) || (num_joints_links == 3))
assert(length(length_links) == num_joints_links)
assert(length(mass_links) == num_joints_links)
assert(length(com_links) == num_joints_links)
assert(length(init_angle) == num_joints_links)
assert(length(init_pos_hip) == 3)
assert(all(com_links <= length_links))
assert(all(size(init_pos_hip)==[3,1]))
assert(all(size(init_angle)==[num_joints_links,1]))

assert((init_angle(1) > pi) || (init_angle(1) < 2*pi))
if num_joints_links == 2
    assert((init_angle(2) > 0.0) || (init_angle(2) < pi))
else
    assert((init_angle(2) > pi) || (init_angle(2) < 2*pi))
    assert((init_angle(3) > 0.0) || (init_angle(3) < pi))
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
%   = {p_01,p_02,p_03,p_04} [3 joints] -> inputs {q1, q1q2, q1q2q3, q1q2q3}

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
                                 forward_kinematics{3});


% Get DYNAMICS of our model.
% Get the torque vector of all joints as function handles, which take a
% q-vector, qdot-vector and qdotdot-vector as input.
% ONLY IMPLEMENTED FOR A 2-LINK SYSTEM
% returns a cell containing:
%   {tau1(q1

dynamics           = get_dynamics(length_links, ...
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
% + vertical drop (implemented in pybullet)
% + gait jumping (implemented in pybullet)
% returns a cell containing:
%   {time vector,
%    q1 vector, q1_dot vector, q1_dotdot vector,
%    q2 vector, q2_dot vector, q2_dotdot vector}

if isfile("trajectory_dpendulum.mat")
    load trajectory_dpendulum.mat jtrajectories_dpendulum
else
    jtrajectories_dpendulum = testcase_double_pendulum(sim_time, time_step);
    save trajectory_dpendulum.mat jtrajectories_dpendulum
end

%jtrajectories_vdrop = testcase_vertical_drop(sim_time, time_step);

%jtrajectories_gjump = testcase_gait_jumping(sim_time, time_step);



%% Torque profiles
% Get the torque profiles for the specified trajectories and compare them
% to the specifications of the Open Dynamics motors.

torque_profiles = get_torques(jtrajectories_dpendulum, dynamics);



%% Visualize
% Visualize test cases.

%plotting(jtrajectories_dpendulum{1}, torque_profiles);
animation(jtrajectories_dpendulum, torque_profiles, length_links, forward_kinematics)
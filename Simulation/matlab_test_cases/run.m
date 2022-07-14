clc; clear all;

% current TODO:
% + impement dynamics & trajectory generation
% + test inverse kinematics (mostly done, it works :) )



%% Initialization

%--------------------------------------------------------------------------
%Change parameters ONLY here!!

num_joints_links    = 2;            %number of joints and linkages (2 or 3)
num_actuated_joints = 2;            %number of actuated joints
length_links   = [1.0, 1.0];        %length of linkages
mass_links     = [5.0, 5.0];        %masses of linkages
com_links      = [0.5, 0.5];        %CoM of linkage i in x-dir of frame i
init_angle     = [deg2rad(270); 
                  deg2rad(90)];     %initial angle of joints
init_pos_hip   = [0.0; 
                  2.0;
                  init_angle(1)];   %initial position of base-frame
length_virtual = 1.5;               %resting length of virtual leg (SLIP)
mass_body      = 50.0;              %mass of the body

%--------------------------------------------------------------------------

% assertions for checking if input parameters are logically correct
assert((num_joints_links == 2) || (num_joints_links == 3))
assert(num_actuated_joints <= num_joints_links)
assert(length(length_links) == num_joints_links)
assert(length(mass_links) == num_joints_links)
assert(length(com_links) == num_joints_links)
assert(length(init_angle) == num_joints_links)
assert(length(init_pos_hip) == 3)
assert(all(com_links <= length_links))
assert((length_virtual > 0) && (length_virtual < sum(length_links)))
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

test = inverse_kinematics([0.0; -1.0], init_angle);


% Get DYNAMICS of our model.
dynamics           = get_dynamics(forward_kinematics, ...
                                  length_links, ...
                                  mass_links, ...
                                  com_links);



%% Trajectories
% Different test cases generate different trajectories. Trajectories in our
% case have to be calculated "by hand" with some logic. The SLIP model
% simulations can help coming up with trajectories for a given test case.

trajectories = get_test_cases();



%% Torque profiles
% Get the torque profiles for the specified trajectories and compare them
% to the specifications of the Open Dynamics motors.

torque_profiles = test_trajectories(trajectories);


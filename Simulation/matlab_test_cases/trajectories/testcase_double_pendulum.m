function joint_trajectories = testcase_double_pendulum(sim_time, time_step)


% q1 \in [180, 360]
% q2 \in [  0, 180]

time       = 0:time_step:sim_time;
time       = time(1:end-mod(length(time_step),4));
time_split = reshape(time,4,[]);
q1_traj    = zeros(size(time_split));
q2_traj    = zeros(size(time_split));
quarter_time_length = length(time_split(1,:));


% first quarter of time:
%   q1: 180 -> 270
%   q2:   0 ->  90
q1_traj(1,:) = linspace(pi, 1.5*pi, quarter_time_length);
q2_traj(1,:) = linspace(0, 0.5*pi, quarter_time_length);

% second quarter of time:
%   q1: 270 -> 360
%   q2:  90 -> 135
q1_traj(2,:) = linspace(1.5*pi, 2*pi, quarter_time_length);
q2_traj(2,:) = linspace(0.5*pi, 0.75*pi, quarter_time_length);

% third quarter of time:
%   q1: 360 -> 315
%   q2: 135 ->  45
q1_traj(3,:) = linspace(2*pi, 1.75*pi, quarter_time_length);
q2_traj(3,:) = linspace(0.75*pi, 0.25*pi, quarter_time_length);

% fourth quarter of time:
%   q1: 315 -> 180
%   q2:  45 ->   0
q1_traj(4,:) = linspace(1.75*pi, pi, quarter_time_length);
q2_traj(4,:) = linspace(0.25*pi, 0, quarter_time_length);


% reshape joint trajectories back into one vector
q1_traj = reshape(q1_traj.',1,[]);
q2_traj = reshape(q2_traj.',1,[]);
% differentiate to angular velocities
q1dot_traj = diff(q1_traj) ./ diff(time);
q2dot_traj = diff(q2_traj) ./ diff(time);

% smooth joint trajectories
q1_smooth = smoothdata(q1_traj,'gaussian');
q2_smooth = smoothdata(q2_traj,'gaussian');
% smooth angular velocity trajectories
q1dot_smooth = smoothdata(q1dot_traj,'gaussian');
q2dot_smooth = smoothdata(q2dot_traj,'gaussian');

% differentiate to angular velocities
q1dotdot_traj = diff(q1dot_smooth) ./ diff(time(1:end-1));
q2dotdot_traj = diff(q2dot_smooth) ./ diff(time(1:end-1));

% smooth angular acceleration trajectories
q1dotdot_smooth = smoothdata(q1dotdot_traj,'gaussian');
q2dotdot_smooth = smoothdata(q2dotdot_traj,'gaussian');


% return
joint_trajectories = {time(1:end-2), ...
                      q1_smooth(1:end-2), ...
                      q1dot_smooth(1:end-1), ...
                      q1dotdot_smooth, ...
                      q2_smooth(1:end-2), ...
                      q2dot_smooth(1:end-1), ...
                      q2dotdot_smooth};
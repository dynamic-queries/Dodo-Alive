function joint_trajectories = testcase_vertical_drop(length_links, ...
                                             mass_links, ...
                                             com_links, ...
                                             mass_toe, ...
                                             mass_body, ...
                                             init_angles, ...
                                             inverse_kinematics, ...
                                             forward_kinematics)

if isfile("../matlab_SLIP_simulation/ws_SLIP_single.mat")
    load ../matlab_SLIP_simulation/ws_SLIP_single.mat x_list t_list
else
    error("Vertical jump trajectory file not found.")
end

assert(length(mass_links) == 3)
assert(length(com_links) == 3)
assert(length(length_links) == 3)

% set up CoM trajectory cell-vector
x_phases = x_list{3};
t_phases = t_list{3};

num_phases          = length(x_phases);
hip_trajectory      = {num_phases};
toe_trajectory      = {num_phases};
hip_toe_disposition = {num_phases};

% com for all links is l/2 (variable com not implemented)
l1       = length_links(1);
l2       = length_links(2);
l3       = length_links(3);
ml1      = mass_links(1);
ml2      = mass_links(2);
ml3      = mass_links(3);
mt       = mass_toe;
mb       = mass_body;
q1_0     = init_angles(1);
q2_0     = init_angles(2);
q3_0     = 2*pi - init_angles(1);         % q3 coupled with q1!!
flight_v = abs(sin(q1_0)*l1 + sin(q1_0+q2_0)*l2 + sin(q1_0+q2_0+q3_0)*l3);

% get CoM offset in resting length
denom         = ml1 + ml2 + ml3 + mt + mb;
flight_offset = abs((...
    sin(q1_0)*(l1/2)*ml1 + ...
    (sin(q1_0)*l1+sin(q1_0+q2_0)*(l2/2))*ml2 + ...
    (sin(q1_0)*l1+sin(q1_0+q2_0)*l2+sin(q1_0+q2_0+q3_0)*(l3/2))*ml3 + ...
    (sin(q1_0)*l1+sin(q1_0+q2_0)*l2+sin(q1_0+q2_0+q3_0)*l3)*mt ...
    ) / denom);

% get SLIP leg length d0 and CoM offset based on SLIP leg length
assert(flight_v > flight_offset)
d0     = flight_v - flight_offset;
offset = @(d)(d * (flight_offset/(flight_v-flight_offset)));

% get toe- and hip- trajectory loop
for p = 1:num_phases
    phase_type = mod(p,2);      % 1 flight, 0 stance
    com_traj = x_phases{p};
    
    % flight phase
    if phase_type
        for i = 1:length(com_traj)
            hip_trajectory{p}(i,1) = com_traj(i,2) + flight_offset;
            toe_trajectory{p}(i,1) = com_traj(i,2) - d0;
            hip_toe_disposition{p}(i,1) = (-1) * (flight_offset + d0);
        end

    % stance phase
    else
        for j = 1:length(com_traj)
            d = com_traj(j,2);
            tmp = offset(d);
            hip_trajectory{p}(j,1) = com_traj(j,2) + offset(d);
            toe_trajectory{p}(j,1) = 0.0;
            hip_toe_disposition{p}(j,1) = (-1) * hip_trajectory{p}(j);
        end
    end
end

% put all important trajectories from cell into vector
hip     = [];
ht_disp = [];
time    = [];
for p = 1:num_phases
    hip     = [hip; hip_trajectory{p}];
    ht_disp = [ht_disp; hip_toe_disposition{p}];
    time    = [time; t_phases{p}];
end

% get joint trajectories
q1    = [];
q2    = [];
q3    = [];
q_old = init_angles;
for i = 1:length(time)
    if time(i) == 0.57652
        check = 5;
    end
    q = inverse_kinematics([0; ht_disp(i)], q_old);
    q_old = q;

    q1 = [q1; q(1)];
    q2 = [q2; q(2)];
    q3 = [q3; 2*pi - q(1)];
end


% refine joint trajectories

% smooth joint trajectories
q1_smooth = smoothdata(q1,'gaussian');
q2_smooth = smoothdata(q2,'gaussian');
q3_smooth = smoothdata(q3,'gaussian');

% differentiate to angular velocities
q1d = diff(q1_smooth) ./ diff(time);
q2d = diff(q2_smooth) ./ diff(time);
q3d = diff(q3_smooth) ./ diff(time);

% smooth angular velocity trajectories
q1d = medfilt1(q1d, 20);
q1d_smooth = smoothdata(q1d,'gaussian');
q2d = medfilt1(q2d, 20);
q2d_smooth = smoothdata(q2d,'gaussian');
q3d = medfilt1(q3d, 20);
q3d_smooth = smoothdata(q3d,'gaussian');

% differentiate to angular accelerations
q1dd = diff(q1d_smooth) ./ diff(time(1:end-1));
q2dd = diff(q2d_smooth) ./ diff(time(1:end-1));
q3dd = diff(q3d_smooth) ./ diff(time(1:end-1));

% smooth angular acceleration trajectories
q1dd = medfilt1(q1dd, 20);
q1dd_smooth = smoothdata(q1dd,'gaussian');
q2dd = medfilt1(q2dd, 20);
q2dd_smooth = smoothdata(q2dd,'gaussian');
q3dd = medfilt1(q3dd, 20);
q3dd_smooth = smoothdata(q3dd,'gaussian');


% return
joint_trajectories = {time(1:end-2), ...
                      hip(1:end-2), ...
                      q1_smooth(1:end-2), ...
                      q1d_smooth(1:end-1), ...
                      q1dd_smooth, ...
                      q2_smooth(1:end-2), ...
                      q2d_smooth(1:end-1), ...
                      q2dd_smooth, ...
                      q3_smooth(1:end-2), ...
                      q3d_smooth(1:end-1), ...
                      q3dd_smooth};

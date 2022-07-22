function torque_profiles = get_torques_vdrop(trajectories, dynamics)


time = trajectories{1};
hip  = trajectories{2};

q1   = trajectories{3};
q1d  = trajectories{4};
q1dd = trajectories{5};
q2   = trajectories{6};
q2d  = trajectories{7};
q2dd = trajectories{8};
q3   = trajectories{9};
q3d  = trajectories{10};
q3dd = trajectories{11};

tau1 = dynamics{1};
tau2 = dynamics{2};

tau1_traj = zeros(length(time),1);
tau2_traj = zeros(length(time),1);

% TODO: IMPLEMENT TORQUE PROFILES FOR VDROP
%for i = 1:length(time)
%    tau1_traj(i) = tau1(q1(i),q1d(i),q1dd(i),q2(i),q2d(i),q2dd(i));
%    tau2_traj(i) = tau2(q1(i),q1d(i),q1dd(i),q2(i),q2d(i),q2dd(i));
%end

torque_profiles = {tau1_traj, tau2_traj};
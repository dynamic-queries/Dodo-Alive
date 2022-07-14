function torque_profiles = get_torques(trajectories, dynamics)


time = trajectories{1};

q1   = trajectories{2};
q1d  = trajectories{3};
q1dd = trajectories{4};
q2   = trajectories{5};
q2d  = trajectories{6};
q2dd = trajectories{7};

tau1 = dynamics{1};
tau2 = dynamics{2};

tau1_traj = zeros(length(time),1);
tau2_traj = zeros(length(time),1);

for i = 1:length(time)
    tau1_traj(i) = tau1(q1(i),q1d(i),q1dd(i),q2(i),q2d(i),q2dd(i));
    tau2_traj(i) = tau2(q1(i),q1d(i),q1dd(i),q2(i),q2d(i),q2dd(i));
end

torque_profiles = {tau1_traj, tau2_traj};
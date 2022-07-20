function joint_trajectories = testcase_double_pendulum(sim_time, time_step)

if isfile("trajectory_dpendulum.mat")
    load ws_SLIP_single.mat x_list t_list
else
    error("Vertical jump trajectory file not found.")
end


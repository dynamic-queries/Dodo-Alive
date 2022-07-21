function joint_trajectories = testcase_double_pendulum(length_links, ...
                                             mass_links, ...
                                             com_links, ...
                                             mass_toe, ...
                                             mass_body, ...
                                             init_angle, ...
                                             inverse_kinematics)

if isfile("trajectory_dpendulum.mat")
    load ws_SLIP_single.mat x_list t_list
else
    error("Vertical jump trajectory file not found.")
end

x_phases = x_list{3};
t_phases = t_list{3};

hip_trajectory = func_com_to_hip(x_phases, length_links(1), mass_links(1), mass_toe, mass_body, init_angle);
toe_trajectory = func_get_toe(x_phases, length_links(1), mass_links(1), mass_toe, mass_body, init_angle);

hip_toe_disposition = [];


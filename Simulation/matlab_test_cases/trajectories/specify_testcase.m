function joint_trajectories = specify_testcase(mode, num_jl, ...
                                               sim_time, time_step, ...
                                               length_links, ...
                                               mass_links, ...
                                               com_links, ...
                                               mass_toe, ...
                                               mass_body, ...
                                               init_angle, ...
                                               inverse_kinematics)


switch mode

    case 'dpend'
        if num_jl == 3
            error("Testcase Double-Pendulum not possible with 3 joints/links.")
        end
        if isfile("trajectories/trajectory_dpendulum.mat")
            load trajectory_dpendulum.mat jtrajectories_dpendulum
        else
            jtrajectories_dpendulum = testcase_double_pendulum(sim_time, ...
                                                               time_step);
            save trajectory_dpendulum.mat jtrajectories_dpendulum
        end

        % return joint trajectories for testcase "Double-Pendulum"
        joint_trajectories = jtrajectories_dpendulum;


    case 'vdrop'
        if num_jl == 2
            error("Testcase Vertical-Drop not possible with 2 joints/links.")
        end
        if isfile("trajectories/trajectory_vdrop.mat")
            load trajectory_vdrop.mat jtrajectories_vdrop
        else
            jtrajectories_vdrop = testcase_vertical_drop(length_links, ...
                                                         mass_links, ...
                                                         com_links, ...
                                                         mass_toe, ...
                                                         mass_body, ...
                                                         init_angle, ...
                                                         inverse_kinematics);
            save trajectory_vdrop.mat jtrajectories_vdrop
        end
        
        % return joint trajectories for testcase "Vertical-Drop"
        joint_trajectories = jtrajectories_vdrop;


    case 'gjump'
        if num_jl == 2
            error("Testcase Gait-Jumping not possible with 2 joints/links.")
        end
        error("Testcase Gait-Jumping not implemented.")

    otherwise
        error("No valid testcase specified.")

end
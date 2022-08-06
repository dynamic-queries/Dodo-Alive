function plot_joint_trajectories(trajectories)


switch length(trajectories)

    case 7
        time      = trajectories{1};
        q1_traj   = trajectories{2};
        q2_traj   = trajectories{5};

        hold on
        plot(time, q1_traj, 'Linewidth', 2)
        plot(time, q2_traj, 'Linewidth', 2)
        hold off
        grid
        legend("q_1 trajectory", "q_2 trajectory")
        

    case 11
        time      = trajectories{1};
        hip       = trajectories{2};
        q1_traj   = trajectories{3};
        q2_traj   = trajectories{6};
        q3_traj   = trajectories{9};

        hold on
        plot(time, q1_traj, 'Linewidth', 2)
        plot(time, q2_traj, 'Linewidth', 2)
        plot(time, q3_traj, 'Linewidth', 2)
        hold off
        grid
        legend("q_1 trajectory", "q_2 trajectory", "q_3 trajectory")

    otherwise
        error("No valid testcase specified.")


end
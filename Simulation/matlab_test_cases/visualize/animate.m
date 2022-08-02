function animate(mode, show_tau, ...
                 trajectories, ...
                 torques, ...
                 length_links, ...
                 forward_kinematics)


switch mode

    case 'dpend'
        animate_dpend(show_tau, trajectories, torques, length_links, forward_kinematics)

    case 'vdrop'
        animate_vdrop(show_tau, trajectories, torques, forward_kinematics)

    case 'gjump'
        error("Testcase Gait-Jumping not implemented.")

    otherwise
        error("No valid testcase specified.")

end
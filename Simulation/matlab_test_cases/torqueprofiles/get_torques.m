function torques = get_torques(mode, joint_trajectories, dynamics)

switch mode

    case 'dpend'
        torques = get_torques_dpend(joint_trajectories, dynamics);

    case 'vdrop'
        torques = get_torques_vdrop(joint_trajectories, dynamics);

    case 'gjump'
        error("Testcase Gait-Jumping not implemented.")

    otherwise
        error("No valid testcase specified.")

end
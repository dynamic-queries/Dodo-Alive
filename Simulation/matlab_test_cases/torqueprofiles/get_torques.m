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


% save results into a mat-file
saved_name.(mode) = torques;
if isfile('resulting_torque_profiles.mat')
    save('resulting_torque_profiles.mat', '-struct', 'saved_name', '-append')
else
    save('resulting_torque_profiles.mat', '-struct', 'saved_name')
end
function animate_dpend(show_tau, trajectories, torques, length_links, forward_kin)

time      = trajectories{1};
q1_traj   = trajectories{2};
q2_traj   = trajectories{5};
tau1_traj = torques{1};
tau2_traj = torques{2};

p01 = forward_kin{2};
p02 = forward_kin{3};

figure
hold on
x_init = linspace(0.0, -length_links(1)-length_links(2), 50);
y_init = linspace(0.0, 0.0, 50);
axis([-0.5, 0.5, -0.5, 0.5])
t = plot(x_init, y_init, 'LineWidth', 1.5);
s = plot(x_init, y_init, 'LineWidth', 1.5);
for i = 1:length(time)
    knee = p01(q1_traj(i),q2_traj(i));
    knee = knee(1:2);
    toe  = p02(q1_traj(i),q2_traj(i));
    toe  = toe(1:2);

    thigh_x = linspace(0.0, knee(1), 50);
    thigh_y = linspace(0.0, knee(2), 50);

    shank_x = linspace(knee(1), toe(1), 50);
    shank_y = linspace(knee(2), toe(2), 50);

    set(t, 'XData', thigh_x, 'YData', thigh_y, 'LineWidth', 1.5)
    set(s, 'XData', shank_x, 'YData', shank_y, 'LineWidth', 1.5)

    if show_tau
        tau1 = tau1_traj(i);
        tau2 = tau2_traj(i);
        str = strcat("\tau_1: ", num2str(tau1), "      \tau_2: ", num2str(tau2));
        a = annotation("textbox",[.2 .5 .3 .3],"String",str,'FitBoxToText','on');
        pause(0.01)
        delete(a)
    else
        pause(0.01)
    end
end

legend("Thigh", "Shank")

hold off

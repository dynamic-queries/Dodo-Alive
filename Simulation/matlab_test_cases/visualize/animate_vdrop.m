function animate_vdrop(show_tau, trajectories, torques, forward_kin)

time      = trajectories{1};
hip       = trajectories{2};
q1_traj   = trajectories{3};
q2_traj   = trajectories{6};
q3_traj   = trajectories{9};
tau1_traj = torques{1};
tau2_traj = torques{2};
l1     = 0.18;
l2     = 0.18;
l3     = 0.18;
q1 = q1_traj(1);
q2 = q2_traj(1);
q3 = q3_traj(1);

p01 = forward_kin{2};   %knee
p02 = forward_kin{3};   %ankle
p03 = forward_kin{4};   %toe

figure
hold on
axis([-1.1, 1.1, 0.0, 2.2])

x_init = linspace(0.0, cos(q1)*l1, 50);
y_init = linspace(hip(1), hip(1)+sin(q1)*l1, 50);
t = plot(x_init, y_init, 'LineWidth', 1.5);

x_init = linspace(cos(q1)*l1, cos(q1)*l1+cos(q1+q2)*l2, 50);
y_init = linspace(hip(1)+sin(q1)*l1, hip(1)+sin(q1)*l1+sin(q1+q2)*l2, 50);
s = plot(x_init, y_init, 'LineWidth', 1.5);

x_init = linspace(cos(q1)*l1+cos(q1+q2)*l2, cos(q1)*l1+cos(q1+q2)*l2+cos(q1+q2+q3)*l3, 50);
y_init = linspace(hip(1)+sin(q1)*l1+sin(q1+q2)*l2, hip(1)+sin(q1)*l1+sin(q1+q2)*l2+sin(q1+q2+q3)*l3, 50);
f = plot(x_init, y_init, 'LineWidth', 1.5);

for i = 1:length(time)
    knee  = p01(q1_traj(i),q2_traj(i));
    knee  = knee(1:2);
    ankle = p02(q1_traj(i),q2_traj(i));
    ankle = ankle(1:2);
    toe   = p03(q1_traj(i),q2_traj(i));
    toe   = toe(1:2);

    thigh_x = linspace(0.0, knee(1), 50);
    thigh_y = linspace(hip(i), hip(i)+knee(2), 50);

    shank_x = linspace(knee(1), ankle(1), 50);
    shank_y = linspace(hip(i)+knee(2), hip(i)+ankle(2), 50);

    foot_x  = linspace(ankle(1), toe(1), 50);
    foot_y  = linspace(hip(i)+ankle(2), hip(i)+toe(2), 50);

    set(t, 'XData', thigh_x, 'YData', thigh_y, 'LineWidth', 1.5)
    set(s, 'XData', shank_x, 'YData', shank_y, 'LineWidth', 1.5)
    set(f, 'XData',  foot_x, 'YData',  foot_y, 'LineWidth', 1.5)
    
    if show_tau
        tau1 = tau1_traj(i);
        tau2 = tau2_traj(i);
        str = strcat("\tau_1: ", num2str(tau1), "      \tau_2: ", num2str(tau2));
        a = annotation("textbox",[.2 .5 .3 .3],"String",str,'FitBoxToText','on');
        pause(0.001)
        delete(a)
    else
        pause(0.001)
    end
end

legend("Thigh", "Shank", "Toe")

hold off
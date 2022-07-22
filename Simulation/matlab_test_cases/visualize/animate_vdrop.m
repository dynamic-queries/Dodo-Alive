function animate_vdrop(trajectories, torques, forward_kin)

time      = trajectories{1};
hip       = trajectories{2};
q1_traj   = trajectories{3};
q2_traj   = trajectories{6};
q3_traj   = trajectories{9};
%tau1_traj = torques{1};
%tau2_traj = torques{2};

p01 = forward_kin{2};   %knee
p02 = forward_kin{3};   %ankle
p03 = forward_kin{4};   %toe

figure
hold on
x_init = linspace(0.0, 0.0, 50);
y_init = linspace(0.0, 0.0, 50);
axis([-0.5, 0.5, 0.0, 3.0])
t = plot(x_init, y_init, 'LineWidth', 1.5);
s = plot(x_init, y_init, 'LineWidth', 1.5);
f = plot(x_init, y_init, 'LineWidth', 1.5);
for i = 1:length(time)
    knee  = p01(q1_traj(i),q2_traj(i));
    knee  = knee(1:2);
    ankle = p02(q1_traj(i),q2_traj(i));
    ankle = ankle(1:2);
    toe   = p03(q1_traj(i),q2_traj(i));
    toe   = toe(1:2);

    thigh_x = linspace(0.0, knee(1), 50);
    thigh_y = linspace(hip(i), hip(i)-knee(2), 50);

    shank_x = linspace(knee(1), ankle(1), 50);
    shank_y = linspace(hip(i)-knee(2), hip(i)-ankle(2), 50);

    foot_x  = linspace(ankle(1), toe(1), 50);
    foot_y  = linspace(hip(i)-ankle(2), hip(i)-toe(2), 50);

    set(t, 'XData', thigh_x, 'YData', thigh_y, 'LineWidth', 1.5)
    set(s, 'XData', shank_x, 'YData', shank_y, 'LineWidth', 1.5)
    set(f, 'XData',  foot_x, 'YData',  foot_y, 'LineWidth', 1.5)
    
    %tau1 = tau1_traj(i);
    %tau2 = tau2_traj(i);
    %str = strcat("\tau_1: ", num2str(tau1), "      \tau_2: ", num2str(tau2));
    %a = annotation("textbox",[.2 .5 .3 .3],"String",str,'FitBoxToText','on');

    pause(0.001)
    %delete(a)
end

legend("Thigh", "Shank")

hold off
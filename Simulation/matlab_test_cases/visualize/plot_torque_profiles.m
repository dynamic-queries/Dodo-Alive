function plot_torque_profiles(time, torque_profiles)


hold on
plot(time, torque_profiles{1}, 'Linewidth', 2)
plot(time, torque_profiles{2}, 'Linewidth', 2)
hold off
grid
legend("\tau_1", "\tau_2")

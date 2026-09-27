t_ref = squeeze(j2_trajectory.Time);
y_ref = squeeze(j2_trajectory.Data);

t_actual = squeeze(theta_out.Time);
y_actual = squeeze(theta_out.Data);

figure

plot(t_ref, rad2deg(y_ref), 'LineWidth', 1.5)
hold on
plot(t_actual, rad2deg(y_actual), 'LineWidth', 1.5)

grid on
xlabel('Time (s)')
ylabel('Joint Angle (degrees)')
legend('Reference','Actual')
title('J2 Trajectory Tracking')
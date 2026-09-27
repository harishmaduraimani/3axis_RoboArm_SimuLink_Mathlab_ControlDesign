t = 0:0.01:5;

theta_ref = zeros(size(t));

theta_ref(t >= 1 & t < 2) = deg2rad(15);
theta_ref(t >= 2 & t < 3) = deg2rad(30);
theta_ref(t >= 3) = deg2rad(45);

j2_trajectory = timeseries(theta_ref,t);
wh
time_span = [0 1];
seed = 1;
v = 30;


rng(seed)
x_max = v*time_span(2);
num_steps = floor(x_max/w.track_period);
steps = [0 (rand(1, num_steps)-0.5) * 2 * w.track_drift_incremental];
track_drift = cumsum(steps);
track_drift_breaks = x_max .* (0:num_steps)/num_steps/v;

figure
stairs(track_drift_breaks, track_drift)
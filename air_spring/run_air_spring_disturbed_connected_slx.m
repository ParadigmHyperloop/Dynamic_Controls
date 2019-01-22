slot_length  = 0.00635; %0.25" [m]
slot_depth   = -0.0127;  %0.5"  [m]
track_length = 3.81;    %12.5' [m]

%track drift
track_drift_max = 0.01016; %0.4" (need to check these numbers)
track_drift_incremental = 0.001016; %0.04"

v = 26.8;               %60 mph

total_length = slot_length + track_length;

period = total_length/v;

wth_pulse_width = 100*slot_length/total_length;
wth_pulse_phase_delay = track_length/v;


p = AirSpringPlant1d_disturbed_2;
%c = disturbance_input_2(p, v);
d = load_system('disturbance_visualization_reduced.slx');
w_handle = SimulinkModelHandle('disturbance_visualization_reduced');
w_model = SimulinkModel(w_handle);

w_model = w_model.setOutputFrame(p.getInputFrame);

connection = struct();
connection(1).from_output = 1;
connection(1).to_input = 1;
connection(2).from_output = 2;
connection(2).to_input = 2;
connection(3).from_output = 3;
connection(3).to_input = 3;
sys = mimoCascade(w_model, p);


time_span = [0, 1];
ytraj = simulate(sys, time_span);



%c = c.setOutputFrame(p.getInputFrame);
%p = p.setOutputFrame(c.getInputFrame);
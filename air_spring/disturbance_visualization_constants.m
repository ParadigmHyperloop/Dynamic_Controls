slot_length  = 0.00635; %0.25" [m]
slot_depth   = -0.0127;  %0.5"  [m]
track_length = 3.81;    %12.5' [m]

%track drift
track_drift_max = 0.01016; %0.4" (need to check these numbers)
track_drift_incremental = 0.001016; %0.04"


%slot_length = 1;
%track_length = 4;
%v = 26.8;               %60 mph
%v = 80;
%v = 60
v = 178;

total_length = slot_length + track_length;

period = total_length/v;

wth_pulse_width = 100*slot_length/total_length;
wth_pulse_phase_delay = track_length/v;
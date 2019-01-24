function u = disturbed_controlled_output(x, K, w, x0, u0)
%DISTURBED_CONTROLLED_OUTPUT 
%Figure out the linear/otherwise controller response to the pod on the
%track. The effective height is x(1) - w(1) = x0.
effective_height = (x(1) - w(1));

ux = u0(1) + K*(effective_height - x0);
uw = w;

u = [ux; uw];

end


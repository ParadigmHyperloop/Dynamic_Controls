function [ mass_out ] = mass_flow_out( pp, pa, zz,  gamma, L, R, T )
%MASS_FLOW_OUT mass flow rate out of region (kg/s)
%   pp: pressure 
%   pa: atmospheric pressure
%   zz: ride height
%   gamma: specific heat ratio
%   L:  perimeter
%   R: molar gas constant
%   T: Temperature

outside_term = pp.*L.*zz/sqrt(R*T);

vout2_term = 2*gamma/(gamma-1) * ((pa./pp).^(2/gamma) - (pa./pp).^((gamma+1)/gamma));

mass_out = outside_term .* sqrt(vout2_term);

end


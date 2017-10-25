%% Flow calculations
%constants of the system:

%environmental constants
pa = 101325; %atmpsoheric pressure [kPa]
gamma = 1.4; %specific heat ratio [1]
T = 300;     %outside temperature [K]
R = 287;     %air gas constant [J/(kg*K)]
g = 9.81;    %gravitational acceleration m/s^2

N = 4;       %number of skates

%Pod 2
% 
% %pod/skate properties
% M = 909;     %mass of pod, [kg]
% %M = 324.319;
% 
% %length/width/area
% L = 0.9144; % length of skate [m]
% %W = 0.127; % width  of skate [m]
% W = 0.3048;  %skate width [m]

M = 5;
L = 0.1397;
W = 0.0762;

A_skate = W*L;    %skate area, [m^2]
Per_skate = 2*(W+L); %perimeter, [m]

porosity = 1;
%porosity = 0.04;

A = porosity * A_skate*N;
Per = Per_skate*N;

h_skate = 0.0445; %bezel height
h_bag = 0.00812;
%h = h_skate + h_bag;
h = h_bag;
V = W*L*h;

%Find equilibrium states
%each skate needs to hold up 1/4 of pod's weight

%zstar = 1.2e-4; %pod 2
%zstar = 1.52e-4; %pod 3

%zstar = 1.52e-4;
%zstar = 2.01e-4; %pod 2, porosity = 1
%zstar = 6.38e-4;
zstar = 5e-5;


pstar = pa + (M*g)/A; %skate pressure, Pa
minstar = porosity * m_out(pstar, pa, zstar, gamma, Per, R, T);

%start mass flow calculations over grid
Ng = 71;
%z = linspace(0, 0.005, N);
z = linspace(0, 5*zstar, Ng);
p = linspace(pa, pstar + 2.5*(pstar - pa), Ng);
[zz, pp] = meshgrid(z, p);

mass_out = m_out( pp, pa, zz, gamma, Per, R, T );

%unit conversions
kgs_to_scfm = 1800.24; %kg/s to standard cubic feet per minute
flow_out = mass_out .*  kgs_to_scfm;

fstar = minstar*kgs_to_scfm;


%% Visualizations
%contour plot 2d + 3d surface
figure(48)
clf
%surf(pp, zz, mass_out)
hold on
%contour(pp, zz, flow_out, 15, 'ShowText', 'on');
surf(pp, zz, flow_out, 'EdgeColor', 'None');
alpha 0.5
contour(pp, zz, flow_out, 20);
plot3([pa, pstar], [zstar, zstar], [fstar, fstar], '--k')
plot3([pstar, pstar], [0, zstar], [fstar, fstar], '--k')
plot3([pstar, pstar], [zstar, zstar], [0, fstar], '--k')
scatter3(pstar, zstar, fstar, 300, '*r')
hold off
xlabel('skate pressure (Pa)')
ylabel('ride height (m)')
zlabel('skate volume flow out (scfm)')
title('Q/skate vs. z and p')
c = colorbar;
c.Label.String = 'Per Skate Flow (scfm)';
view(3)

%contour plot 2d only
figure(49)
clf
hold on
%surf(pp, zz, flow_out)
contour(pp, zz, flow_out, 10, 'ShowText', 'on')
scatter(pstar, zstar, 100, '*r')
txtZ = strcat('z^* = ', num2str(zstar*1000, 2), ' mm');
txtP = strcat('p^* = ', num2str(pstar/1000, 4), ' kPa');
txtQ = strcat('Q_{in} = ', num2str(fstar, 4), ' scfm');
text(pa, zstar - 3e-5, txtZ)
text(pstar + 1e2, 0+ 3e-5, txtP)
text(pstar + 2e2, zstar,txtQ)
plot([pstar, pstar], [0, zstar], '--k')
plot([pa, pstar], [zstar, zstar], '--k')
hold off
xlabel('skate pressure (Pa)')
ylabel('ride height (m)')
zlabel('skate volume flow out (scfm)')
title('Q/skate vs. z and p')
c = colorbar;
c.Label.String = 'Per Skate Flow (scfm)';
axis square


function [ mass_out ] = m_out( pp, pa, zz,  gamma, Per, R, T )
%m_out finds the mass flow rate out in an air skate in kg/s

%pp:    skate pressure
%pa:    atmospheric (outside) pressure
%zz:    current ride height
%gamma: specific heat ratio, ~1.4
%Per:   total perimeter of air skates
%R:     air gas constant
%T:     atmospheric temperature

outside_term = pp.*Per.*zz/sqrt(R*T);

vout2_term = 2*gamma/(gamma-1) * ((pa./pp).^(2/gamma) - (pa./pp).^((gamma+1)/gamma));

mass_out = outside_term .* sqrt(vout2_term);

end

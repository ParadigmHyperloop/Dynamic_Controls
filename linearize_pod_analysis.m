use_bag = 0;

if use_bag == 2
    p = BagSideFlowEnginePlant;
elseif use_bag == 1
    p = BagFlowEnginePlant;
else
    p = FlowEnginePlant;
end


[A, B, C, D, xdot0] = p.linearize(0, p.x0, p.u0);
eA = eig(full(A));
stable = all(real(eA) < 0);
% A = full(A);
% B = full(B);
% if use_bag == 2
%     sys = ss(A, B, C, D, 'StateName', {'Height', 'Velocity', 'Bag Pressure', 'Bottom Pressure', 'Side Pressure'},...
%                          'OutputName',{'Height', 'Velocity', 'Bag Pressure', 'Bottom Pressure', 'Side Pressure'},...
%                          'InputName', 'MassFlowIn');
% elseif use_bag == 1
%     sys = ss(A, B, C, D, 'StateName', {'Height', 'Velocity', 'Bag Pressure', 'Bottom Pressure'},...
%                          'OutputName',{'Height', 'Velocity', 'Bag Pressure', 'Bottom Pressure'},...
%                          'InputName', 'MassFlowIn');
% else
%     sys = ss(A, B, C, D, 'StateName', {'Height', 'Velocity', 'Pressure'},...
%                          'OutputName', {'Height', 'Velocity', 'Pressure'},...
%                          'InputName', 'MassFlowIn');
% end
% 
% 
% tfinal = 0.2;
% %tfinal = 15;
% 
% figure(29)
% subplot(1, 4, 1)
% opts = bodeoptions;
% opts.FreqUnits = 'Hz';
% bode(sys, opts)
% 
% 
% subplot(1, 4, 2)
% iopzmap(sys)
% 
% t = linspace(0, tfinal, 201);
% subplot(1, 4, 3)
% impulse(sys, t)
% 
% subplot(1, 4, 4)
% step(sys, t)
% 


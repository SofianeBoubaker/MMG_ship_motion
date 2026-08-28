function [t,y,delta_vec] = solver(data,motion)
% SOLVER: Solve the equation of motion y_dot=f(t,y) with f the function MMG_3DOF
% Inputs:
%   data: Ship parameters and simulation settings
%   motion: 0 for turning test, 1 for zig-zag test
% Outputs:
%   t: Time vector (s)
%   y: State variables [u, v, r] over time at the center of gravity
%   delta_vec: Rudder angle history (deg)



% Input validation
if ~ismember(motion, [0, 1])
    error('motion must be 0 (turning) or 1 (zig-zag)');
end

%% Initialization
t     = [0];
y     = [data.U0 0 0]; % Initial speed conditions 
dtR    = data.dtR;    %rudder execution time (s)

delta_max = data.delta_max; % [deg] Maximum rudder deflection
delta = data.delta0;
delta_vec = [delta];

%% Simulation loop
while t(end)<data.T_final
    
    %phase 1 : ramp rudder angle to delta_max
    while abs(delta)<=abs(delta_max) && t(end)<data.T_final
    % Set the time span used to solve the equation of motion (the rudder angle is constant on this interval)
        if (t(end)+dtR)>=data.T_final        %t must never exceed T_final
            tspan = [t(end) data.T_final];
        else
            tspan   = [t(end) t(end)+dtR]; 
        end
        Eta     = [y(end,1) y(end,2) y(end,3)]; % Initial conditions of state variables 
        [tt,yt] = ode45(@(t, Z) MMG_3DOF(t, Z, data, delta), tspan, Eta); %ODEs solving at a fixed delta
        %append results
        t(end+1:end+length(tt))   = tt;
        y(end+1:end+length(tt),:) = yt;
        delta_vec(end+1:end+length(tt)) = delta * ones(1,length(tt));
        delta = delta+sign(delta_max)*data.delta_speed * min(dtR,(data.T_final-t(end))); % Rudder angle increased if delta_max>0/decreased if delta_max<0
        clear tt yt
    end
    
    clear tspan 
    psi_current = trapz(t,y(:,3)); %last heading angle
    delta = delta_max;

    %phase 2 : motion with a constant rudder angle
    % For turning (motion=0): Always runs once (condition is always true)
    % For zig-zag (motion=1): Runs until absolute heading reaches delta_max
   
    while (motion*sign(delta_max)*psi_current*180/pi)<abs(delta_max) && t(end)<data.T_final 
        
        if motion==0 || ((t(end)+dtR)>=data.T_final)
            tspan = [t(end) data.T_final];
        else
            tspan = [t(end) t(end)+dtR];
        end
        Eta   = [y(end,1) y(end,2) y(end,3)]; 
        [tc,yc] = ode45(@(t, Z) MMG_3DOF(t, Z, data, delta), tspan, Eta); %ODEs solving
        %append results
        t(end+1:end+length(tc))   = tc;
        y(end+1:end+length(tc),:) = yc;
        delta_vec(end+1:end+length(tc)) = delta_max * ones(1,length(tc));
        psi_current = trapz(t,y(:,3));
    end
    % prepare the next zigzag cycle
    delta_max = - delta_max; %targeted rudder direction flipped 
    delta = delta+sign(delta_max)*data.delta_speed * min(dtR,data.T_final-t(end)); %rudder starts moving
end


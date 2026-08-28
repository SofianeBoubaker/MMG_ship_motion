function [t,y,delta_vec] = solver(data,motion)

% Input validation
if ~ismember(motion, [0, 1])
    error('motion must be 0 (turning) or 1 (zig-zag)');
end

t     = [0];
y     = [data.U0 0 0]; % Initial conditions 
dtR    = data.dtR;    %rudder execution time (s)

delta_max = data.delta_max; % [deg] Maximum rudder deflection
delta = data.delta0;
delta_vec = [delta];

while t(end)<data.T_final
    
    %phase 1
    while abs(delta)<=abs(delta_max) && t(end)<data.T_final
        if (t(end)+dtR)>=data.T_final        %t must never exceed T_final
            tspan = [t(end) data.T_final];
        else
            tspan   = [t(end) t(end)+dtR]; 
        end
        Eta     = [y(end,1) y(end,2) y(end,3)]; % Initial conditions of state variables for rudder deflection
        [tt,yt] = ode45(@(t, Z) MMG_3DOF(t, Z, data, delta), tspan, Eta);
        t(end+1:end+length(tt))   = tt;
        y(end+1:end+length(tt),:) = yt;
        delta_vec(end+1:end+length(tt)) = delta * ones(1,length(tt));
        delta = delta+sign(delta_max)*data.delta_speed * min(dtR,(data.T_final-t(end))); % Rudder angle increased if delta_max>0/decreased if delta_max<0
        clear tt yt
    end
    
    clear tspan 
    psi_current = trapz(t,y(:,3));
    delta = delta_max;

    %phase 2
    while (motion*sign(delta_max)*psi_current*180/pi)<abs(delta_max) && t(end)<data.T_final %only 1 loop if turning (motion=0)
        
        if motion==0 || ((t(end)+dtR)>=data.T_final)
            tspan = [t(end) data.T_final];
        else
            tspan = [t(end) t(end)+dtR];
        end
        Eta   = [y(end,1) y(end,2) y(end,3)]; 
        [tc,yc] = ode45(@(t, Z) MMG_3DOF(t, Z, data, delta), tspan, Eta);
        t(end+1:end+length(tc))   = tc;
        y(end+1:end+length(tc),:) = yc;
        psi_current = trapz(t,y(:,3));
        delta_vec(end+1:end+length(tc)) = delta_max * ones(1,length(tc));
    end

    delta_max = - delta_max;
    delta = delta+sign(delta_max)*data.delta_speed * min(dtR,data.T_final-t(end));
end


function [t,y,delta_vec] = solver(data,motion)

t     = [0];
y     = [data.U_initial 0 0]; % Initial conditions 
tr    = data.dt_r;    %rudder execution time (s)
if ~isfield(data, 't') || tr == 0 tr=1;    %set to 1 s if tr not specified in the file
end
delta_max = data.delta_c; % [deg] Maximum rudder deflection
delta = data.delta_initial;
delta_vec = [delta];

while t(end)<data.T_final
    
    %phase 1
    while abs(delta)<=abs(delta_max) && t(end)<data.T_final
        if (t(end)+tr)>=data.T_final        %t must never exceed T_final
            tspan = [t(end) data.T_final];
        else
            tspan   = [t(end) t(end)+tr]; 
        end
        Eta     = [y(end,1) y(end,2) y(end,3)]; % Initial conditions of state variables for rudder deflection
        [tt,yt] = ode45(@(t, Z) MMG_3DOF(t, Z, data, delta), tspan, Eta);
        t(end+1:end+length(tt))   = tt;
        y(end+1:end+length(tt),:) = yt;
        delta_vec(end+1:end+length(tt)) = delta * ones(1,length(tt));
        delta = delta+sign(delta_max)*data.delta_star * min(tr,(data.T_final-t(end))); % Rudder angle increased if delta_max>0/decreased if delta_max<0
        clear tt yt
    end
    
    clear tspan delta
    psi = trapz(t,y(:,3));
    delta = delta_max;

    %phase 2
    while (motion*sign(delta_max)*psi*180/pi)<abs(delta_max) && t(end)<data.T_final %only 1 loop if turning (motion=0)
        
        if motion==0 || ((t(end)+tr)>=data.T_final)
            tspan = [t(end) data.T_final];
        else
            tspan = [t(end) t(end)+tr];
        end
        Eta   = [y(end,1) y(end,2) y(end,3)]; 
        [tc,yc] = ode45(@(t, Z) MMG_3DOF(t, Z, data, delta), tspan, Eta);
        t(end+1:end+length(tc))   = tc;
        y(end+1:end+length(tc),:) = yc;
        psi = trapz(t,y(:,3));
        delta_vec(end+1:end+length(tc)) = delta_max * ones(1,length(tc));
    end

    delta_max = - delta_max;
    delta = delta+sign(delta_max)*data.delta_star * min(tr,data.T_final-t(end));
end


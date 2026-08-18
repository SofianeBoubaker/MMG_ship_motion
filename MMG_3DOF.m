function Z_dot = MMG_3DOF(~,Z,data,delta)

delta = delta * pi/180;
u = Z(1);
v = Z(2);
r = Z(3);

vm = v - data.LCG * r;
beta = atan(-vm/u);
U = sqrt(u^2 + vm^2);
Fdim  = 0.5*data.rho*data.L*data.d*U^2;%dimensionless factor (N)
Ndim  = Fdim * data.L;
Mdim  = 0.5*data.rho*data.L^2*data.d;
Idim  = Mdim * data.L^2;

m     = data.rho * data.C_B * data.L * data.B * data.d;
Iz    = m * (0.25*data.L)^2; %approximation
M     = [m+data.m_x*Mdim 0 0; 0 m+data.m_y*Mdim data.LCG*m; 0 data.LCG*m Iz+data.LCG^2*m+data.J_z*Idim]; 

%non-dimensional derivatives coefficients
X0    =  abs(data.X_0); %convention x0>0 ensures that the resistance opposes the surge motion because XH=-X0<0
Xvv   =  data.X_vv;
Xvvvv =  data.X_vvvv;
Xrr   =  data.X_rr;
Xvr   =  data.X_vr;
Yv    =  data.Y_v;
Yvvv  =  data.Y_vvv;
Yr    =  data.Y_r;
Yrrr  =  data.Y_rrr;
Yvrr  =  data.Y_vrr; 
Yvvr  =  data.Y_vvr;
Nv    =  data.N_v;
Nvvv  =  data.N_vvv;
Nr    =  data.N_r;
Nrrr  =  data.N_rrr;
Nvrr  =  data.N_vrr;
Nvvr  =  data.N_vvr;

vm_p  =  vm/U;
r_p   =  r * data.L/U;

X_H = Fdim * (-X0 + Xvv * vm_p^2 + Xvr * vm_p * r_p + Xrr * r_p^2 + Xvvvv * vm_p^4);
Y_H = Fdim * (Yv * vm_p + Yr * r_p + Yvvv * vm_p^3 + Yvvr * vm_p^2 * r_p + Yvrr * vm_p * r_p^2 + Yrrr * r_p^3);
N_H = Ndim * (Nv * vm_p + Nr * r_p + Nvvv * vm_p^3 + Nvvr * vm_p^2 * r_p + Nvrr * vm_p * r_p^2 + Nrrr * r_p^3);

if (data.nb_rud) == 1 
    
    beta_p = beta - data.x_p/data.L * r_p;
    w_p = data.w_p0 * exp(-4*beta_p^2);
    %w_p = 1 - (1 - data.w_p0) * (1 + (1 - np.cos(beta_p)^2 * (1 - np.abs(beta_p)));
    %w_p = 1 - (1 - data.w_p0) * (1 + (1 - exp(-C1 * abs(beta_p))) * (C2 - 1));
    %%For the KVLCC1, C1 = 2 ,C2 = 1.6 (if beta_p>0) or C2 = 1.1 (if beta_p<0) 
    
    J_p = u * (1 - w_p) / (data.n_p * data.D);
    K_T = data.k_0 + data.k_1 * J_p + data.k_2 * J_p^2;
    T_p = data.rho * data.n_p^2 * data.D^4 * K_T;
    X_p = (1 - data.t_p) * T_p;
    
    beta_R = beta - data.l_R * r_p;
    if  beta_R > 0
        gamma_R = data.gamma_plus;
    else
        gamma_R = data.gamma_minus;
    end
    v_R = U * gamma_R * beta_R;
    eta = data.D/data.H;
    u_R = data.epsilon * u * (1 - w_p) * sqrt(eta * (1 + data.kapa * (sqrt(1 + 8 * K_T/pi/J_p^2) - 1))^2 + (1 - eta));
    alpha_R = delta - atan(v_R/u_R);
    U_R = sqrt(u_R^2+ v_R^2);
    f_alpha = data.lambda * 6.13 / (data.lambda +  2.25);
    Fn = 0.5 * data.rho * data.A_R * U_R^2 * f_alpha * sin(alpha_R);
    
    X_R = -(1 - data.t_R) * Fn * sin(delta);
    Y_R = -(1 + data.a_H) * Fn * cos(delta);
    N_R = -(data.x_R + data.a_H * data.x_H) * Fn * cos(delta);
    
elseif (data.nb_rud) == 2
    %The formula are not implemented for 2 rudder in this assignment
end

FX = ( X_H + X_R + X_p ) + vm * M(2,2) * r + r^2 * data.LCG * m;
FY = ( Y_H + Y_R ) - M(1,1) * u * r;
FN = ( N_H + N_R ) - data.LCG * m * u * r;
F  = [FX;FY;FN];

Z_dot = M \ F;
Z_dot(2) = Z_dot(2) + data.LCG * Z_dot(3); %v_dot = vm_dot + Xg * r_dot


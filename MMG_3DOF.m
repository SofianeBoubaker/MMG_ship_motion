function Z_dot = MMG_3DOF(~,Z,data,delta_deg)

delta = delta_deg * pi/180;
u = Z(1);
v = Z(2);
r = Z(3);

vm = v - data.xG * r;
beta = atan(-vm/u);
U = sqrt(u^2 + vm^2);
Fdim  = 0.5*data.rho*data.L*data.d*U^2;%dimensionless factor (N)
Ndim  = Fdim * data.L;
Mdim  = 0.5*data.rho*data.L^2*data.d;
Idim  = Mdim * data.L^2;

m     = data.rho * data.CB * data.L * data.B * data.d;
Iz    = m * (0.25*data.L)^2; %approximation
M     = [m+data.mx_prime*Mdim 0 0; 0 m+data.my_prime*Mdim data.xG*m; 0 data.xG*m Iz+data.xG^2*m+data.Jz_prime*Idim]; 

%non-dimensional derivatives coefficients
X0_prime    =  abs(data.X0_prime); %convention x0>0 ensures that the resistance opposes the surge motion because XH=-X0<0
Xvv_prime   =  data.Xvv_prime;
Xvvvv_prime =  data.Xvvvv_prime;
Xrr_prime   =  data.Xrr_prime;
Xvr_prime   =  data.Xvr_prime;
Yv_prime    =  data.Yv_prime;
Yvvv_prime  =  data.Yvvv_prime;
Yr_prime    =  data.Yr_prime;
Yrrr_prime  =  data.Yrrr_prime;
Yvrr_prime  =  data.Yvrr_prime; 
Yvvr_prime  =  data.Yvvr_prime;
Nv_prime    =  data.Nv_prime;
Nvvv_prime  =  data.Nvvv_prime;
Nr_prime    =  data.Nr_prime;
Nrrr_prime  =  data.Nrrr_prime;
Nvrr_prime  =  data.Nvrr_prime;
Nvvr_prime  =  data.Nvvr_prime;

vm_prime  =  vm/U;
r_prime   =  r * data.L/U;

XH = Fdim * (-X0_prime + Xvv_prime * vm_prime^2 + Xvr_prime * vm_prime * r_prime + Xrr_prime * r_prime^2 + Xvvvv_prime * vm_prime^4);
YH = Fdim * (Yv_prime * vm_prime + Yr_prime * r_prime + Yvvv_prime * vm_prime^3 + Yvvr_prime * vm_prime^2 * r_prime + Yvrr_prime * vm_prime * r_prime^2 + Yrrr_prime * r_prime^3);
NH = Ndim * (Nv_prime * vm_prime + Nr_prime * r_prime + Nvvv_prime * vm_prime^3 + Nvvr_prime * vm_prime^2 * r_prime + Nvrr_prime * vm_prime * r_prime^2 + Nrrr_prime * r_prime^3);

if (data.nb_rud) == 1 
    
    betaP = beta - data.xP/data.L * r_prime;
    if isfield(data,'wP0')
        if isfield(data,'C1') || isfield(data,'C2Minus') || isfield(data,'C2Plus')
            if betaP > 0
                C2 = data.C2Plus;
            else
                C2 = data.C2Minus;
            end
            wP = 1 - (1 - data.wP0) * (1 + (1 - exp(-data.C1 * abs(betaP))) * (C2 - 1));
        else
            wP = data.wP0 * exp(-4*betaP^2);
        end
    elseif isfield(data,'wP')
        wP = data.wP;
    else 
        error('wP0 or wP are missing')
    end      
           
    JP = u * (1 - wP) / (data.nP * data.DP);
    KT = data.k0 + data.k1 * JP + data.k2 * JP^2;
    TP = data.rho * data.nP^2 * data.DP^4 * KT;
    XP = (1 - data.tP) * TP;
    
    betaR = beta - data.lR_prime * r_prime;
    if  betaR > 0
        gammaR = data.gammaPlus;
    else
        gammaR = data.gammaMinus;
    end
    vR = U * gammaR * betaR;
    eta = data.DP/data.HR;
    uR = data.epsilon * u * (1 - wP) * sqrt(eta * (1 + data.kapa * (sqrt(1 + 8 * KT/pi/JP^2) - 1))^2 + (1 - eta));
    alphaR = delta - atan(vR/uR);
    UR = sqrt(uR^2+ vR^2);
    falpha = data.lambda * 6.13 / (data.lambda +  2.25);
    Fn = 0.5 * data.rho * data.AR * UR^2 * falpha * sin(alphaR);
    
    if ~isfield(data,'xR_prime')
        data.('xR_prime') = -0.5;
    end
    
    XR = -(1 - data.tR) * Fn * sin(delta);
    YR = -(1 + data.aH) * Fn * cos(delta);
    xR = data.xR_prime *  data.L;
    xH = data.xH_prime * data.L;
    NR = -(xR + data.aH * xH) * Fn * cos(delta);
    
elseif (data.nb_rud) == 2
    error("The formula are not implemented for 2 rudders yet");
end

FX = ( XH + XR + XP ) + vm * M(2,2) * r + r^2 * data.xG * m;
FY = ( YH + YR ) - M(1,1) * u * r;
FN = ( NH + NR ) - data.xG * m * u * r;
F  = [FX;FY;FN];

Z_dot = M \ F;
Z_dot(2) = Z_dot(2) + data.xG * Z_dot(3); %v_dot = vm_dot + Xg * r_dot


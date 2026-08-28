%% MMG 3DOF Ship Maneuvering Simulation - Main Script
% This script simulates ship maneuvering motions (turning or zig-zag) using the MMG 3DOF model.
% It reads input data, runs the simulation, post-processes results, and plots trajectories and maneuvering characteristics.
% Results are shown at midship section

clc

%% User imputs

motion = input('Motion test (0: turning / 1: zigzag): ');
filename = input('input file name ("filename.txt"): '); % to get the input file where the ship caracteristics are stored
data = readFile(filename); % data collection
data.('dtR') = input('rudder execution time step (s): ');
disp('info: rudder angle progresses in steps, increasing of dtR * delta_speed each dtR time ');
data.('T_final') = input('simulation time (s): ');
data.('delta_max') = input('targeted rudder angle (deg): ');
data.('rho') =  1025; %m3/s water density

%% Simulation run 

[t,y,delta] = solver(data,motion);

%% Post-processing

beta  = atan(-y(:,2)./y(:,1)) * 180 / pi; %drfit angle
psi   = cumtrapz(t,y(:,3)); %heading angle
U     = sqrt(y(:,1).^2 + y(:,2).^2); %resultant speed


%ship-fixed velocities at midship in Earth-fixed frame
x_dot    = y(:,1).*cos(psi) - y(:,2).*sin(psi);
y_dot    = y(:,1).*sin(psi) + y(:,2).*cos(psi);

%Earth-fixed positions at midship
x_pos = cumtrapz(t,x_dot); 
y_pos = cumtrapz(t,y_dot);

%Turning test analysis

if motion == 0
    i = 1;
    check = 0; % flag to track progress through the test
    while i <=length(psi) %loop through all time steps
        if abs(psi(i))>=pi/2 && check==0
            check = 1;
            ind_90 = i; %index where psi reach 90 deg
            x_90  = abs(x_pos(ind_90)); %distance traveled in x direction
            y_90  = abs(y_pos(ind_90)); %distance traveled in y direction
            disp(['Advance = ',num2str(x_90),' m'])
            disp(['Transfer = ',num2str(y_90),' m'])

        end
        if abs(psi(i))>=pi && check==1
            ind_180 = i; %index when psi reach 180 deg
            check=2;
            y_180 = abs(y_pos(ind_180)); %lateral displacment at 180 deg
            disp(['Tactical diameter = ',num2str(y_180),' m'])
        end
        if abs(psi(i))>=(3*pi) && check==2
            ind_540 = i; %index where psi reach 540 deg (1.5 full turn)
            check=3;
            y_540 = abs(y_pos(ind_540));
        end
        if abs(psi(i))>=(4*pi) && check==3
            ind_720 = i; %index where psi reach 720 deg (2 full turns)
            check=4;
            y_720 = abs(y_pos(ind_720));
            %results at the end of the simulation
            disp(['Steady turning diameter = ',num2str(y_540-y_720),' m'])
            disp(['Steady yaw rate = ',num2str(y(end,3)*180/pi),' deg/s'])
            disp(['Steady turning speed = ',num2str(U(end)),' m/s'])
        end
        i=i+1;
        if check==4
            break %exit if all characteristics are calculated
        end
    end
 

% Zig-zag test analysis

else
    i = 1;
    check = 0;
    while i <=length(psi) && check<2
        if data.delta_max * y(i,3)<0 && check==0 %checks when the yaw rate and rudder angle signs become opposite (meaning heading angle reaches an extrema)
            OSA_1 = abs(abs(psi(i)*180/pi)-abs(data.delta_max)); 
            check = 1;
            disp(['1st Overshoot Angle = ',num2str(OSA_1),' deg'])
        end
        if data.delta_max * y(i,3)>0 && check==1 %checks when the yaw rate and rudder angle signs become opposite a second time (meaning heading angle reaches an 2nd extrema)
            OSA_2 = abs(abs(psi(i)*180/pi)-abs(data.delta_max));
            check=2;
            disp(['2st Overshoot Angle = ',num2str(OSA_2),' deg']) 
        end
        i=i+1;
    end
       
end

%% Plots at midship

fig=1;
figure(fig)
subplot(2,2,1)
plot(y_pos,x_pos);
title('Midship Trajectory')
xlabel('y [m]')
ylabel('x [m]')
grid on

subplot(2,2,2)
plot(t,U)
title('Midship Speed U')
xlabel('Time [s]')
ylabel('U [m/s]')
grid on

subplot(2,2,3)
plot(t,beta)
title('Midship Drift angle')
xlabel('Time [s]')
ylabel('Beta [deg]')
grid on

subplot(2,2,4)
plot(t,y(:,3)*180/pi)
title('Yaw')
xlabel('Time [s]')
ylabel('Yaw Rate [deg/s]')
grid on

fig=fig+1;
figure(fig)
plot(t,rad2deg(psi),'b')
title('Yaw Angle')
hold on
plot(t,delta,'r--')
xlabel('Time [s]')
ylabel('Yaw/Rudder Angle [deg]')
grid on
legend('Heading angle','Rudder angle');
hold off

fig=fig+1;
figure(fig)
plot(y_pos/data.L,x_pos/data.L);
title('Trajectory at midship')
xlabel('y/L [-]')
ylabel('x/L [-]')
grid on
axis equal



clc

motion = input('Motion test (0: turning / 1: zigzag): ');
filename = input('input file name ("filename.txt"): ');
data = readFile(filename);
data.('dtR') = input('rudder execution time step (s): ');
disp('info: rudder angle progresses in steps, increasing of dtR * delta_speed each dtR time ');
data.('T_final') = input('simulation time (s): ');
data.('delta_max') = input('targeted rudder angle (deg): ');
data.('rho') =  1025; %m3/s water density

[t,y,delta] = solver(data,motion);

vm    = y(:,2) - data.xG * y(:,3);
beta  = atan(-vm./y(:,1)) * 180 / pi;
psi   = cumtrapz(t,y(:,3));
x_dot    = y(:,1).*cos(psi) - y(:,2).*sin(psi);
y_dot    = y(:,1).*sin(psi) + y(:,2).*cos(psi);
U     = sqrt(y(:,1).^2 + vm.^2);

x_pos = cumtrapz(t,x_dot);
y_pos = cumtrapz(t,y_dot);

if motion == 0
    i = 1;
    check = 0;
    while i <=length(psi)
        if abs(psi(i))>=pi/2 && check==0
            check = 1;
            ind_90 = i;
            x_90  = abs(x_pos(ind_90));
            y_90  = abs(y_pos(ind_90));
            disp(['Advance = ',num2str(x_90),' m'])
            disp(['Transfer = ',num2str(y_90),' m'])

        end
        if abs(psi(i))>=pi && check==1
            ind_180 = i;
            check=2;
            y_180 = abs(y_pos(ind_180));
            disp(['Tactical diameter = ',num2str(y_180),' m'])
        end
        if abs(psi(i))>=(3*pi) && check==2
            ind_540 = i;
            check=3;
            y_540 = abs(y_pos(ind_540));
        end
        if abs(psi(i))>=(4*pi) && check==3
            ind_720 = i;
            check=4;
            y_720 = abs(y_pos(ind_720));
            disp(['Steady turning diameter = ',num2str(y_540-y_720),' m'])
            disp(['Steady yaw rate = ',num2str(y(end,3)*180/pi),' deg/s'])
            disp(['Steady turning speed = ',num2str(U(end)),' m/s'])
        end
        i=i+1;
        if check==4
            break
        end
    end
 
else
    i = 1;
    check = 0;
    while i <=length(psi) && check<2
        if data.delta_max * y(i,3)<0 && check==0
            OSA_1 = abs(abs(psi(i)*180/pi)-abs(data.delta_max));
            check = 1;
            disp(['1st Overshoot Angle = ',num2str(OSA_1),' deg'])
        end
        if data.delta_max * y(i,3)>0 && check==1
            OSA_2 = abs(abs(psi(i)*180/pi)-abs(data.delta_max));
            check=2;
            disp(['2st Overshoot Angle = ',num2str(OSA_2),' deg']) 
        end
        i=i+1;
    end
       
end


fig=1;
figure(fig)
subplot(2,2,1)
plot(y_pos,x_pos);
title('Trajectory')
xlabel('y [m]')
ylabel('x [m]')
grid on
% axis equal

subplot(2,2,2)
plot(t,U)
title('Speed U')
xlabel('Time [s]')
ylabel('U [m/s]')
grid on

subplot(2,2,3)
plot(t,beta)
title('Drift angle')
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
title('Trajectory')
xlabel('y/L [-]')
ylabel('x/L [-]')
grid on
axis equal



% GRUPPO 20


clc
clear
%% Simulation Settings

% set each variable to 0 or 1, depending on which disturbance has to be
% simulated
%   _drag:  atmospheric drag
%   _J2:    J2 effect
%   _Mgrav: gravity gradient torque

enable_drag     = 1;
enable_J2       = 1;
enable_Mgrav    = 1;

% set to 1 the variable of the controller that has to be used
%   _SMC:   second order sliding mode control 
%   _PD:    proportional-derivative control
% the other controller's variable must be set to 0

enable_SMC      = 0;
enable_PD       = 1;

if enable_SMC == enable_PD
    error(' (line 27-28): enable_SMC must be different from enable_PD')
end
%% Section 1A: Chaser Data 

%--------------------------------------------------------------------------
% Chaser parameters
%--------------------------------------------------------------------------
l    = 0.6;                                                                % [m]      Chaser dimensions
L    = 1;                                                                  % [m]
m_c0 = 120;                                                                % [kg]     Initial mass                 
Jx0  = (m_c0*(2*l^2)/12);          
Jy0  = (m_c0*(2*L*l)/12);       
Jz0  = (m_c0*(2*L*l)/12);
J0   = [Jx0  0   0;                                                        % [kg*m^2] Inertial tensor
         0  Jy0  0;               
         0   0  Jz0];
J    = J0;
invJ = inv(J);

%--------------------------------------------------------------------------
% PD/SMC Attitude controller (tipo 1 - ES3) parameters
%--------------------------------------------------------------------------

% Controller PD kp:
k   = 1.5;
kp1 = k*eye(3);

% Controller PD kd:
d  = 25; 
kd = d*eye(3);

% Controller SMC second order
lambda = 0.03;
alpha  = 0.01;

%--------------------------------------------------------------------------
%   Guidance law parameters -  APF
%--------------------------------------------------------------------------

% Artificial Potential Field
ka    = 3;                                                                 % Attractive constant
kr    = 10000;                                                             % Repulsive constant
x_obs = [-100 0 0];                                                        % Obstacle position in LVLH
R_obs = 0.1;                                                               % [m]
toll  = 0.05;                                                               % [m]

%--------------------------------------------------------------------------
%   Attitude CI/des
%--------------------------------------------------------------------------
q_0   = [0.5 0.5 0.5 0.5]';                                                % Initial quaternion
q_des = [1 0 0 0]';                                                        % Desired attitude quaternion
e_ss  = 0.02;                                                              % Steady state error
 
 
%%  Section 1B: Thrusts and Moments due to the thrusters and RW 
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Thruster specs.
%--------------------------------------------------------------------------
Isp1  = 220;                                                               % [s]   Specific impulse
c1    = Isp1*9.81;                                                         % [m/s] Dumping speed
Tmax  = 1;                                                                 % [N]   Maximum thrust
n     = 2;                                                                 % [-]   Number of thrusters x axes (Tot. 12, 2 for each side of the 3 directions X,Y,Z body)
F_thr = n*Tmax;                                                            % [N]   Maximum thrust per axes

%--------------------------------------------------------------------------
% PWPF
%--------------------------------------------------------------------------
U_max    = 2;
U_on     = 1;                                                              % Activation treshold
U_off    = 0.29;                                                           % Deactivation treshold
edb      = 0.2;                                                            % Dead band amplitude
kf       = U_max/edb;                                                      % Filter Gain
deltaton = 0.02;                                                           %[s]Minimum on-time
tau      = -deltaton/log((1-(U_on-U_off)/kf));                             %[s]Filter time constant

%--------------------------------------------------------------------------
% RW specs.
%--------------------------------------------------------------------------
gmax   = 0.005;                                                            % [N*m]    RW Maximum torque
m_RW   = 0.6;                                                              % [kg]     RW Mass
d_RW   = 0.095;                                                            % [m]      RW Diameter   
I_RW   = 0.5*m_RW*(d_RW/2)^2;                                              % [kg*m^2] RW Moment of inertia
om_0   = [0,0,0]';
tau_RW = 1.5645e-2;                                                        % [s]      RW  Filter Time constant
p_RW   = 1;                                                                % [rad/s]  RW Pole
Kr     = 1;                                                                %          RW Filter Gain

B       = 30;                                                              % [°]      Angle of RW's pyramidal configuration 
B       = B*pi/180;                                                        % [rad]    Converted angle
ZZZ     = [cos(B)    0      -cos(B)    0;                                  % Conversion matrix for angular momentum/ couples of a pyramidal reaction wheel scheme
             0      cos(B)    0      -cos(B);
           sin(B)   sin(B)   sin(B)   sin(B)];
INV_ZZZ = pinv(ZZZ);

%% Section 2: Errors due to external disturbances
% THE USER CAN CONSIDER THESE ERRORS IN COMBINATION WITH THE DETAILED MODEL
% OF GRAVITY GRADIENT AND J2 EFFECT.
% CONSIDERA FORZE IN ASSI BODY E POI LE PORTA IN LVLH SUL MODELLO SIMULINK.

%--------------------------------------------------------------------------
%   Force due to air drag model
%--------------------------------------------------------------------------
mu     = 3.986012*10^14;
r_E    = 6378.145*10^3;                                                    % [m]      Earth Radius
height = 400*10^3;                                                         % [m]      Reference altitude
r      = r_E + height;                                                     % [m]      Orbita Radius
omega  = (mu/r^3)^0.5;                                                     % [rad]    Target angular velocity
rho    = 1*10^(-12);                                                       % [kg/m^3] Air density obtained as medium value for 500 km orbit Fehse
Vx     = omega*r;                                                          % [m/s]    Orbital velocity
CD     = 2.2;                                                              % [-]      Drag coefficient for satellite of compact shapes
A      = l^L;                                                              % [m^2]    Cross section
F_D    = -1/2*rho*CD*A*Vx^2;                                               % [N]      Force due to the drag

%--------------------------------------------------------------------------
%   J2
%--------------------------------------------------------------------------
J2  = 1.08263*10^-6;
FJ2 = -3*J2*m_c0*mu*r_E^2/(2*r^4);                                         % [N] Force due to oblateness of the Earth

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Section 3: Maneuvers initial conditions
% For each phase a different simulation is defined; the final conditions of
% the previous maneuver are used as initial conditions of the next one
%--------------------------------------------------------------------------    
%  Orbit
%--------------------------------------------------------------------------
mu     = 3.986012*10^14;                                                   % [m^3/s^2] Standard gravitational parameter
g0     = 9.81;                                                             % [m/s^2]   Zero gravity
r_E    = 6378.145*10^3;                                                    % [m]       Earth Radius
height = 400*10^3;                                                         % [m]       Reference altitude
r      = r_E + height;                                                     % [m]       Orbita Radius
omega  = (mu/r^3)^0.5;                                                     % [rad]     Target angular velocity
v      = sqrt(mu/r);                                                       % [m/s]     Tangential orbital speed

%--------------------------------------------------------------------------
% Simulation parameters
%--------------------------------------------------------------------------
tsp_0   = 0.02;                                                            % [s] Zero shoot time

Dt      = 0.01;                                                            % [s] Time step of simulation
Dt_guid = 1;                                                               % [s] Update sample time of the guidance algorithm
Dt_con  = 0.02;                                                            % [s] Update sample time of the control algorithm

t_tot   = 10000;                                                           % [s] Max. time used to run each phase

%% Free Drift
X0     =  -11000;                                                          % [m] Initial X position in LVLH
Y0     = 0;                                                                % [m] Initial Y position in LVLH

zc     = 397*10^3;                                                         % [m] Chaser orbit altitude
zt     = 400*10^3;                                                         % [m] Target orbit altitude
deltaz = zt - zc;                                                          % [m] Difference of orbits altitude between Chaser and Target 
Z0     = deltaz;                                                           % [m] Initial Z position in LVLH
Pos0   = [X0 Y0 Z0]';                                                      % [m] Initial position vector

Vx0    = 3/2*omega*Z0;                                                     % [m/s] Relative speed in LVLH when the Target and Chaser orbits are different
x0_dot = Vx0;                                                              % [m/s] Initial X speed in LVLH 
y0_dot = 0;                                                                % [m/s] Initial Y speed in LVLH 
z0_dot = 0;                                                                % [m/s] Initial Z speed in LVLH
Vel0   = [x0_dot y0_dot z0_dot]';                                          % [m/s] Initial speed vector
xdot0  = x0_dot;

wb_0 = [0;0;0];                                                            % [rad/s] Initial condition of angular velocity
q0_0 = q_0(1);                                                             % Initial quaternion vector
qv_0 = q_0(2:4);

X0_H  = -10250;                                                            % End of Free Drift / Hohmann Start Condition


out = sim('FREE_DRIFT',t_tot);

Pos_FD  = out.Pos;
Fb_FD   = out.Fb;
Vel_FD  = out.Vel;
EA_FD   = out.EulerAngles;
Q_FD    = out.quaternions;
tout_FD = out.tout;
wb_FD   = out.wb;
M_RW4_FD    = out.M_RWRW;
qerr_FD     = out.normqerr;
mgravFD     = out.M_grav;

%% Hohmann Transfer
Pos0   = Pos_FD(end,:)';
Vel0   = Vel_FD(end,:)';
wb_0   = wb_FD(end,:)';
q0_0   = Q_FD(end,1)';
qv_0   = Q_FD(end,2:4)';
m_c0_H = m_c0;
dv_H   = omega/4*Pos0(3);
dt_H   = m_c0_H*(dv_H)/F_thr;

out = sim('HOHMANN',t_tot);

tout_H = out.tout;
Pos_H  = out.Pos;
Vel_H  = out.Vel;
Fb_H   = out.Fb;
EA_H   = out.EulerAngles;
Q_H    = out.quaternions;
wb_H   = out.wb;
m_H    = out.m;
mf_H   = out.mf;

M_RW4_H    = out.M_RWRW;
qerr_H     = out.normqerr;

%% Radial boost
Pos0    = Pos_H(end,:)';
Vel0    = Vel_H(end,:)';
wb_0    = wb_H(end,:)';
q0_0    = Q_H(end,1)';
qv_0    = Q_H(end,2:4)';
m_c0_RB = m_H(end);
X_thr   = -500;
dv_RB   = abs((omega/4)*(Pos0(1)-X_thr))-Vel0(3);                          % [m/s]
dt_RB   = m_c0_RB*(dv_RB)/F_thr;                                           % [s] 
                                                                    
out = sim('RADIAL_BOOST',t_tot);

tout_RB = out.tout;
Pos_RB  = out.Pos;
Vel_RB  = out.Vel;
Fb_RB   = out.Fb;
EA_RB   = out.EulerAngles;
Q_RB    = out.quaternions;
wb_RB   = out.wb;
m_RB    = out.m;
mf_RB   = out.mf;

M_RW4_RB    = out.M_RWRW;
qerr_RB     = out.normqerr;

%% Cone maneuver Custom Law
Pos0     = Pos_RB(end,:)';
Vel0     = Vel_RB(end,:)';
wb_0     = wb_RB(end,:)';
q0_0     = Q_RB(end,1)';
qv_0     = Q_RB(end,2:4)';
m_c0_CUS  = m_RB(end);                                                       
                                                               
Kp = 0.05;                                                                 % Proportional gain for the Y-axis PD controller                                                                
Kd = 0.1;                                                                  % Derivative gain for the Y-axis PD controller
lambda0=atan(Pos0(3)/Pos0(1));
out = sim('FINAL_APPROACH_CUSTOM',t_tot);

tout_CUS = out.tout;
Pos_CUS  = out.Pos;
Vel_CUS  = out.Vel;
Fb_CUS  = out.Fb;
EA_CUS   = out.EulerAngles;
Q_CUS    = out.quaternions;
wb_CUS   = out.wb;
m_CUS    = out.m;
mf_CUS   = out.mf;

M_RW4_CUS    = out.M_RWRW;
qerr_CUS    = out.normqerr;
%--------------------------------------------------------------------------
% Final point chaser Custom Law
%--------------------------------------------------------------------------
pos_f_CUS = Pos_CUS(end,:);                                                  % [m]
xf_CUS    = pos_f_CUS(1);                                                    % [m]  
yf_CUS    = pos_f_CUS(2);                                                    % [m]
zf_CUS    = pos_f_CUS(3);                                                    % [m]

%% Cone maneuver Artificial Potential Field
Pos0     = Pos_RB(end,:)';
Vel0     = Vel_RB(end,:)';
wb_0     = wb_RB(end,:)';
q0_0     = Q_RB(end,1)';
qv_0     = Q_RB(end,2:4)';
m_c0_APF = m_RB(end);

xf       = [-2 0 0]';                                                      % [m]
pos_goal=xf;
pos_obs=x_obs;
xdot_max = 0.1;                                                            % [m/s] Maximum desired speed per axis

nu0= R_obs+toll;                                                           


syms x y z

U_a_sym   = 0.5*ka*(sqrt((pos_goal(1)-x)^2+(pos_goal(2)-y)^2+(pos_goal(3)-z)^2))^2;         % Target position attractive potential
U_r_sym   = 0.5*kr*(1/(sqrt((x-pos_obs(1))^2+(y-pos_obs(2))^2+(z-pos_obs(3))^2))-1/nu0)^2;  % Chaser Position Repulsive potential
U_tot_sym = U_a_sym+U_r_sym;
f_tot_sym = -gradient(U_tot_sym, [x, y, z]);                                                % Symbolic Force field generated by the total potential
f_tot     = matlabFunction(f_tot_sym,'File','create_f_tot');                                % Function handle of Force field generated by the total potential
f_a_sym   = -gradient(U_a_sym, [x,y,z]);                                                    % Symbolic Force field generated by the target attractive potential
f_a       = matlabFunction(f_a_sym,'File','create_f_a');                                    % Function handle of Force field generated by the target attractive potential

%PD costants for APF
Kp=[0.00001 0.05 0.01]';                                                   % Proportional gains for the 3-axis position PD controller
Kd=[1 0.1 1]';                                                               % Derivative gains for the 3-axis position PD controller


out=sim('FINAL_APPROACH_APF',t_tot);

tout_APF = out.tout;
Pos_APF  = out.Pos;
Vel_APF  = out.Vel;
Fb_APF   = out.Fb;
EA_APF   = out.EulerAngles;
Q_APF    = out.quaternions;
wb_APF   = out.wb;
m_APF    = out.m;
mf_APF   = out.mf;

M_RW4_APF    = out.M_RWRW;
qerr_APF     = out.normqerr;
%--------------------------------------------------------------------------
% Final point chaser Artificial Potential Field
%--------------------------------------------------------------------------

pos_f_APF = Pos_CUS(end,:);
xf_APF    = pos_f_APF(1);                                                  % [m]  
yf_APF    = pos_f_APF(2);                                                  % [m]
zf_APF    = pos_f_APF(3);                                                  % [m]

%--------------------------------------------------------------------------    
% Cone geometry
%--------------------------------------------------------------------------    
d1   = 500;                                                                % [m]   Ideal x-distance between target and chaser
r1   = 1;                                                                  % [m]   Half Cone height at initial point 
r2   = 0.1;                                                                % [m]   Half Cone height at final point
teta = atan((r1 - r2)/d1);                                                 % [rad] Inclination angle

xcono=linspace(-500,-2,2);                                                 % Vectors which modelize the cone geometry
zconoup=-0.9/498*xcono+0.0964;
zconodown=-zconoup;

%% PLOT - Trajectory 

%--------------------------------------------------------------------------

% Trajectory XZ Plane
figure(1)

%START/TARGET
p1=plot(0,0,'s','Markersize',5,'Linewidth',2); hold on,p2=plot(Pos_FD(1,1),Pos_FD(1,3),'s','Markersize',5,'Linewidth',2);

%COMPLETE TRAJECTORY
%plot([Pos_FD(:,1);Pos_H(:,1);Pos_RB(:,1)],[Pos_FD(:,3);Pos_H(:,3);Pos_RB(:,3)],'linewidth',1.5), hold on

%TRAJECTORY DIVIDED IN SECTIONS
p3=plot(Pos_FD(:,1),Pos_FD(:,3),'linewidth',1.5);

p4=plot(Pos_H(:,1),Pos_H(:,3),'linewidth',1.5);

p5=plot(Pos_RB(:,1),Pos_RB(:,3),'linewidth',1.5);

%p6=plot(Pos_CUS(:,1),Pos_CUS(:,3),'linewidth',1.5);

p7=plot(Pos_APF(:,1),Pos_APF(:,3),'r','linewidth',1.5);

% CONE GEOMETRY

% plot(xcono,zconoup,'r','linewidth',1);
% plot(xcono,zconodown,'r','linewidth',1);

% OBSTACLE + TOLL

% th = 0:pi/50:2*pi;
% xunit = nu0 * cos(th) + x_obs(1);
% zunit = nu0 * sin(th) + x_obs(3);
% plot(xunit, zunit);

% OBSTACLE

% xunit2 = R_obs * cos(th) + x_obs(1);
% zunit2 = R_obs * sin(th) + x_obs(3);
% plot(xunit2, zunit2,'r');

set(gca, 'XDir','reverse')
set(gca, 'YDir','reverse')
grid on

% FULL TRAJECTORY

xticks([-11000:1000:0])
xlim([-11000 10]), 
ylim([-1000 3000]);

xlabel( 'X_{LVLH} [m]');
ylabel('Z_{LVLH} [m]');
title('Plane V_{bar}- R_{bar}: Position')
legend([p2 p3 p4 p5 p7 p1],{'STARTING POINT','FREE DRIFT','HOHMANN TRANSFER','RADIAL BOOST','FINAL APPROACH (APF)','TARGET'})
%% PLOT - TRAJECTORY (CONE APPROACH)

 figure(2)

 p1=plot(Pos_CUS(:,1),Pos_CUS(:,3),'b','linewidth',1.5);hold on
 p2=plot(Pos_APF(:,1),Pos_APF(:,3),'m','linewidth',1.5);


% % CONE GEOMETRY
 p4=plot(xcono,zconoup,'r','linewidth',1);
 p5=plot(xcono,zconodown,'r','linewidth',1);
 
% %obstacle+TOLL
 th = 0:pi/50:2*pi;
 xunit = nu0 * cos(th) + x_obs(1);
 zunit = nu0 * sin(th) + x_obs(3);
 p6=plot(xunit, zunit,'g','linewidth',1.5);
 
% %obstacle
 xunit2 = R_obs * cos(th) + x_obs(1);
 zunit2 = R_obs * sin(th) + x_obs(3);
 p7=plot(xunit2, zunit2,'k','linewidth',1.5);

 set(gca, 'XDir','reverse')
 set(gca, 'YDir','reverse')
 grid on


xlabel( 'X_{LVLH} [m]');
ylabel('Z_{LVLV} [m]');

% CONE APPROACH
%    xticks([-500:50:0])
%    yticks([-1:0.2:1])
   xlim([-510 2]), ylim([-1 1])

% OBSTACLE VISUALIZATION
% xlim([-101 -99]), ylim([-0.8 0.8])
title('Plane V_{bar}- R_{bar}: Position (Cone Approach)')
legend([p1 p2 p4 p6 p7],{'PROPORTIONAL NAVIGATION','ARTIFICIAL POTENTIAL FIELD','CONE GEOMETRY','OBSTACLE SECURITY TOLLERANCE (APF)','OBSTACLE (APF)'},'Location','southwest')

figure(150)

plot3(Pos_APF(:,1),Pos_APF(:,2),Pos_APF(:,3),'r','linewidth',1)
xlabel( 'X_{LVLH} [m]');
ylabel('Y_{LVLH} [m]')
zlabel('Z_{LVLV} [m]');
hold on
r=R_obs;
[X,Y,Z]=sphere;
X2 = X * r;
Y2 = Y * r;
Z2 = Z * r;
surf(X2-100,Y2,Z2)
title('3D View: Position respect to the obstacle (Cone Approach)')

figure(151)

plot3(Pos_APF(:,1),Pos_APF(:,2),Pos_APF(:,3),'r','linewidth',1)
xlabel( 'X_{LVLH} [m]');
ylabel('Y_{LVLH} [m]')
zlabel('Z_{LVLV} [m]');
hold on
r=nu0;
[X,Y,Z]=sphere;
X2 = X * r;
Y2 = Y * r;
Z2 = Z * r;
surf(X2-100,Y2,Z2)
title('3D View: Position respect to the obstacle+tollerance (Cone Approach)')


%% PLOT -  Velocity V_bar, R_bar over time

tendH = tout_H(end);
tendFD = tout_FD(end);
tendRB = tout_RB(end);
tendCUS = tout_CUS(end);
tendAPF = tout_APF(end);

figure (3)

subplot(5,1,1)

plot([tout_FD;tout_H+tendFD;tout_RB+tendH+tendFD;...
    tout_CUS+tendRB+tendH+tendFD],[Vel_FD(:,1);Vel_H(:,1);Vel_RB(:,1);Vel_CUS(:,1)],'b','linewidth',1.5)
xlabel('t [s]')
h2=ylabel('$\dot{X}_{LVLH}  [m/s]$');
set(h2, 'Interpreter', 'latex');
title('V_{bar}: Velocity')
grid on,
xticks([0:500:12000])
xlim([0,tendRB+tendH+tendFD+tendCUS])

subplot(5,1,2)

plot(tout_APF+tendRB+tendH+tendFD,Vel_APF(:,1),'m','linewidth',1.5), hold on,
plot(tout_CUS+tendRB+tendH+tendFD,Vel_CUS(:,1),'b','linewidth',1.5), legend({'ARTIFICIAL POTENTIAL FIELD','CUSTOM LAW'},'Location','northeast')
xlabel('t [s]')
h2=ylabel('$\dot{X}_{LVLH}  [m/s]$');
set(h2, 'Interpreter', 'latex');
title('V_{bar}: Velocity')
grid on,
xticks([0:500:12000])
xlim([tendRB+tendH+tendFD,tendRB+tendH+tendFD+tendCUS])

subplot(5,1,3)

plot([tout_FD;tout_H+tendFD;tout_RB+tendH+tendFD;...
    tout_CUS+tendRB+tendH+tendFD],[Vel_FD(:,3);Vel_H(:,3);Vel_RB(:,3);Vel_CUS(:,3)],'b','linewidth',1.5)
xlabel('t [s]')
h2=ylabel('$\dot{Z}_{LVLH}  [m/s]$');
set(h2, 'Interpreter', 'latex');
title('R_{bar}: Velocity')
grid on,
xticks([0:500:12000])
xlim([0,tendRB+tendH+tendFD+tendCUS])


subplot(5,1,4)

%plot(tout_APF+tendRB+tendH+tendFD,Vel_APF(:,3),'m','linewidth',1.5), hold on,
plot(tout_CUS+tendRB+tendH+tendFD,Vel_CUS(:,3),'b','linewidth',1.5), legend({'CUSTOM LAW'},'Location','southeast')
xlabel('t [s]')
h2=ylabel('$\dot{Z}_{LVLH}  [m/s]$');
set(h2, 'Interpreter', 'latex');
title('R_{bar} Velocity (Cone Approach)')
grid on,
xticks([0:500:12000])
xlim([tendRB+tendH+tendFD,tendRB+tendH+tendFD+tendCUS])


subplot(5,1,5)

plot(tout_APF+tendRB+tendH+tendFD,Vel_APF(:,3),'m','linewidth',1.5), hold on,
%plot(tout_CUS+tendRB+tendH+tendFD,Vel_CUS(:,3),'b','linewidth',1.5), 
legend({'ARTIFICIAL POTENTIAL FIELD'},'Location','southeast')
xlabel('t [s]')
h2=ylabel('$\dot{Z}_{LVLH}  [m/s]$');
set(h2, 'Interpreter', 'latex');
title('R_{bar} Velocity (Cone Approach)')
grid on,
xticks([0:500:12000])
xlim([tendRB+tendH+tendFD,tendRB+tendH+tendFD+tendCUS])

%% PLOT - Position and velocity H_BAR over time

tendH = tout_H(end);
tendFD = tout_FD(end);
tendRB = tout_RB(end);
tendCUS = tout_CUS(end);
tendAPF = tout_APF(end);

figure (4)

subplot(5,1,1)

plot([tout_FD;tout_H+tendFD;tout_RB+tendH+tendFD;...
    tout_CUS+tendRB+tendH+tendFD],[Pos_FD(:,2);Pos_H(:,2);Pos_RB(:,2);Pos_CUS(:,2)],'b','linewidth',1.5)
xlabel('t [s]')
h1=ylabel('${Y}_{LVLH} [m]$');
set(h1, 'Interpreter', 'latex');
title('H_{bar} Position')
grid on
xticks([0:1000:12000])
xlim([0,tendRB+tendH+tendFD+tendCUS])

subplot(5,1,2)

plot(tout_APF+tendRB+tendH+tendFD,Pos_APF(:,2),'m','linewidth',1.5), hold on,
plot(tout_CUS+tendRB+tendH+tendFD,Pos_CUS(:,2),'b','linewidth',1.5), legend({'ARTIFICIAL POTENTIAL FIELD','CUSTOM LAW'},'Location','southeast')
xlabel('t [s]')
h1=ylabel('${Y}_{LVLH} [m]$');
set(h1, 'Interpreter', 'latex');
title('H_{bar} Position (Cone Approach)')
grid on
xticks([0:500:12000])
xlim([tendRB+tendH+tendFD,tendRB+tendH+tendFD+tendCUS])

subplot(5,1,3)

plot([tout_FD;tout_H+tendFD;tout_RB+tendH+tendFD;...
    tout_CUS+tendRB+tendH+tendFD],[Vel_FD(:,2);Vel_H(:,2);Vel_RB(:,2);Vel_CUS(:,2)],'b','linewidth',1.5)
xlabel('t [s]')
h2=ylabel('$\dot{Y}_{LVLH}  [m/s]$');
set(h2, 'Interpreter', 'latex');
title('H_{bar} Velocity')
grid on,
xticks([0:1000:12000])
xlim([0,tendRB+tendH+tendFD+tendCUS])

subplot(5,1,4)

%plot(tout_APF+tendRB+tendH+tendFD,Vel_APF(:,2),'m','linewidth',1.5), hold on,
plot(tout_CUS+tendRB+tendH+tendFD,Vel_CUS(:,2),'b','linewidth',1.5), legend({'CUSTOM LAW'},'Location','southeast')
xlabel('t [s]')
h2=ylabel('$\dot{Y}_{LVLH}  [m/s]$');
set(h2, 'Interpreter', 'latex');
title('H_{bar} Velocity (Cone Approach)')
grid on,
xticks([0:500:12000])
xlim([tendRB+tendH+tendFD,tendRB+tendH+tendFD+tendCUS])

subplot(5,1,5)

plot(tout_APF+tendRB+tendH+tendFD,Vel_APF(:,2),'m','linewidth',1.5), hold on,
%plot(tout_CUS+tendRB+tendH+tendFD,Vel_CUS(:,2),'b','linewidth',1.5), 
legend({'ARTIFICIAL POTENTIAL FIELD'},'Location','southeast')
xlabel('t [s]')
h2=ylabel('$\dot{Y}_{LVLH}  [m/s]$');
set(h2, 'Interpreter', 'latex');
title('H_{bar} Velocity (Cone Approach)')
grid on,
xticks([0:500:12000])
xlim([tendRB+tendH+tendFD,tendRB+tendH+tendFD+tendAPF])

%% PLOT - Thrusters x y z 

figure(5)
sgtitle('Thrust: Free Drift > Radial Boost')
subplot(3,1,1)
plot([tout_FD;tout_H+tendFD;tout_RB+tendH+tendFD],[zeros(length(tout_FD),1);Fb_H(:,1);Fb_RB(:,1)],'b','linewidth',1.5)
ylim([-2 2]),xlim([0,tendFD+tendH+tendRB]),grid on
xlabel('t [s]'),ylabel('F_{X B} [N]')

subplot(3,1,2)
plot([tout_FD;tout_H+tendFD;tout_RB+tendH+tendFD],[zeros(length(tout_FD),1);Fb_H(:,2);Fb_RB(:,2)],'b','linewidth',1.5)
ylim([-2 2]),xlim([0,tendFD+tendH+tendRB]), grid on
xlabel('t [s]'),ylabel('F_{Y B} [N]')

subplot(3,1,3)
plot([tout_FD;tout_H+tendFD;tout_RB+tendH+tendFD],[zeros(length(tout_FD),1);Fb_H(:,3);Fb_RB(:,3)],'b','linewidth',1.5)
ylim([-2 2]),xlim([0,tendFD+tendH+tendRB]), grid on
xlabel('t [s]'),ylabel('F_{Z B} [N]')

figure(6)
sgtitle('Thrust: Cone Approach (Custom Law)')

subplot(3,1,1)
plot(tout_CUS+tendRB+tendH+tendFD,Fb_CUS(:,1),'b','LineWidth',1.5)
grid on,ylim([-2 2]),xlim([tendFD+tendH+tendRB,tendFD+tendH+tendRB+tendCUS])
xlabel('t [s]'),ylabel('F_{X B} [N]')

subplot(3,1,2)
plot(tout_CUS+tendRB+tendH+tendFD,Fb_CUS(:,2),'b','LineWidth',1.5)
grid on,ylim([-2 2]),xlim([tendFD+tendH+tendRB,tendFD+tendH+tendRB+tendCUS])
xlabel('t [s]'),ylabel('F_{Y B} [N]')

subplot(3,1,3)
plot(tout_CUS+tendRB+tendH+tendFD,Fb_CUS(:,3),'b','LineWidth',1.5)
grid on,ylim([-2 2]),xlim([tendFD+tendH+tendRB,tendFD+tendH+tendRB+tendCUS])
xlabel('t [s]'),ylabel('F_{Z B} [N]')

figure(7)
sgtitle('Thrust: Cone Approach (Artificial Potential Field)')

subplot(3,1,1)
plot(tout_APF+tendRB+tendH+tendFD,Fb_APF(:,1),'b','LineWidth',1.5)
grid on,ylim([-2 2]),xlim([tendFD+tendH+tendRB,tendFD+tendH+tendRB+tendAPF])
xlabel('t [s]'),ylabel('F_{X B} [N]')

subplot(3,1,2)
plot(tout_APF+tendRB+tendH+tendFD,Fb_APF(:,2),'b','LineWidth',1.5)
grid on,ylim([-2 2]),xlim([tendFD+tendH+tendRB,tendFD+tendH+tendRB+tendAPF])
xlabel('t [s]'),ylabel('F_{Y B} [N]')

subplot(3,1,3)
plot(tout_APF+tendRB+tendH+tendFD,Fb_APF(:,3),'b','LineWidth',1.5)
grid on,ylim([-2 2]),xlim([tendFD+tendH+tendRB,tendFD+tendH+tendRB+tendAPF])
xlabel('t [s]'),ylabel('F_{Z B} [N]')

%% PLOT - Attitude Control

 tendH = tout_H(end);
 tendFD = tout_FD(end);
 tendRB = tout_RB(end);
 tendCUS = tout_CUS(end);
 tendAPF = tout_APF(end);

figure(8)

subplot(2,1,1)
plot(tout_FD,Q_FD), hold on
plot(tout_FD,qerr_FD,'LineWidth',2), grid on
legend('q0','q1','q2','q3','||q_{err}||')
xlabel('t [s]'), ylim([-0.2 1.2])
title('Quaternions and errors (Focus on Free Drift)')

subplot(2,1,2)
plot([tout_FD;tout_H+tendFD;tout_RB+tendH+tendFD;...
    tout_CUS+tendRB+tendH+tendFD],[Q_FD;Q_H;Q_RB;Q_CUS]), hold on
plot([tout_FD;tout_H+tendFD;tout_RB+tendH+tendFD;tout_CUS+tendRB+tendH+tendFD]...
    ,[qerr_FD;qerr_H;qerr_RB;qerr_CUS],'LineWidth',2), grid on
legend('q0','q1','q2','q3','||q_{err}||')
xlabel('t [s]'), ylim([-0.2 1.2])
title('Quaternions and errors (Free Drift > Cone Approach CUS)')

figure(9)

subplot(2,1,1)
plot(tout_FD,M_RW4_FD,'LineWidth',2)
legend('RW1','RW2','RW3','RW4')
title('Momentum RWs (Focus on Free Drift) ')
ylabel('Torque [Nm]'), xlabel('t [s]'), grid on
ylim([-0.006 0.006])

subplot(2,1,2)
plot([tout_FD;tout_H+tendFD;tout_RB+tendH+tendFD;...
    tout_CUS+tendRB+tendH+tendFD],[M_RW4_FD;M_RW4_H;M_RW4_RB;M_RW4_CUS])
legend('RW1','RW2','RW3','RW4')
title('Momentum RWs (Free Drift > Cone approach CUS)')
ylabel('Torque [Nm]'), xlabel('t [s]'), grid on
ylim([-0.006 0.006])

figure(10)

subplot(2,1,1)
plot(tout_FD,wb_FD(:,1),'LineWidth',2), hold on
plot(tout_FD,wb_FD(:,2),'LineWidth',2), hold on
plot(tout_FD,wb_FD(:,3),'LineWidth',2)
legend('\omega_{bx}','\omega_{by}','\omega_{bz}'), grid on
xlabel('t [s]')
ylabel('[rad/s]')
title('\omega_b (Focus on Free Drift)')

subplot(2,1,2)

for i = 1:3
    plot([tout_FD;tout_H+tendFD;tout_RB+tendH+tendFD;tout_CUS+tendRB+tendH+tendFD]...
        ,[wb_FD(:,i);wb_H(:,i);wb_RB(:,i);wb_CUS(:,i)]), hold on
end
legend('\omega_{bx}','\omega_{by}','\omega_{bz}'), grid on
xlabel('t [s]')
ylabel('[rad/s]')
title('\omega_b (Free drift > Cone approach CUS)')
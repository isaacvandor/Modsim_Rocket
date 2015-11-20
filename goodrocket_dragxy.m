function res = goodrocket_dragxy(theta, velocity)
height = height_func(velocity, theta);
res = height;

function res = height_func(velocity, theta)
theta = theta*pi/180;
[vx, vy] = pol2cart(theta, velocity); %velocities in m/s
options = odeset('MaxStep', 1e-2, 'RelTol', 1e-15, 'AbsTol', 1e-15,'Events', @events_func);
[~,M] = ode45(@slope_func, [0, 200], [0, 1, vx, vy], options);
X = M(:,1);
Y = M(:,2);
plot(X,Y)
%axis([0 14000 0 8000])
hold on
xlabel('x Position of Good Missile (m)')
ylabel('y Position of Good Missile (m)')
title('Trajectory of Intercepting Missile given 500 m/s Launch Velocity')
res = Y(end); % the result is the final Y position in meters
end
function [value,isterminal,direction] = events_func(t, W)
% stop the integration when the good rocket gets to the max range of Iron Dome.
max_range = 70000; % Max range of Iron dome in meters
value = [W(1) - max_range;W(2)]; % equals 0 when you hit the max range of Iron Dome
isterminal = [1;1];
direction = [1;-1];
end
function res = slope_func(t, W)
% this is a slope function invoked by ode45
% W contains 4 elements, Px, Py, Vx, and Vy
P = W(1:2); % position of the good rocket in meters
V = W(3:4); % velocity of the good rocket in m/s
dPdt = V;
dVdt = acceleration_func(t, P, V);
res = [dPdt; dVdt];
end
function res = acceleration_func(t,P,V)
Ag = [0; -9.8];
initial_mass = 90; %rocket mass in kg
fuel_mass = 40; %kg
dry_mass = 10;
burn_rate = 3.71; %kg/s
%mass = ((dry_mass + fuel_mass)/dry_mass);
%total_mass = (dry_mass + fuel_mass) - burn_Rate*t;
Ad = drag_force_func(V) / initial_mass; %m/s^2
res = Ag+Ad;
end
function Fd = drag_force_func(V)
Cd = .4;
rho = 1.225;
shadow_area = .2;
v = norm(V); %magnitude of velocity in m/s
% Fdx = cos(theta)*Cd*((rho*v^2)/2)*shadow_area;
% Fdy = sin(theta)*Cd*((rho*v^2)/2)*shadow_area;
Fd = cos(theta)*Cd*(((rho*v^2)/2)*shadow_area)+sin(theta)*Cd*(((rho*v^2)/2)*shadow_area);
end
end

% g = 9.81;%gravitational acceleration
% Cl = 0.5; %lift coefficient LIES
% exhaust_area = .05; % LIES
% exhaust_velocity = 1200; %LIES?
% atmospheric_pressure = 101325;%LIES?
% exhaust_pressure = 45000;
% wing_area = .2;
% Fgy = mass*g;
% Ftx = cos(theta)*((((exhaust_pressure - atmospheric_pressure)*exhaust_area)/burnrate + exhaust_velocity)*log(massratio));
% Fty = sin(theta)*((((exhaust_pressure - atmospheric_pressure)*exhaust_area)/burnrate + exhaust_velocity)*log(massratio));
% Flx = cos(theta)* Cl * ((rho*v^2)/2)*wing_area;
% Fly = sin(theta)* Cl * ((rho*v^2)/2)*wing_area;
% 
% Fx = Ftx - Fdx + Flx;
% Fy = -Fgy + Fty - Fdy + Fly;
% dvxdt = Fx./mass;
% dvydt = Fy./mass;
% res = [vx; vy; dvxdt; dvydt];

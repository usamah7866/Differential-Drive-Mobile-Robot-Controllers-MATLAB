%% Starting control with ADRC dynamic controller
clear
clc
N = 310;                 % Number of Iterations
T = 0.1;                 % Sampling time (sec.)

Vr = 0.25;               % Reference Linear velocity (m/s)
Wr = 0.5;                % Reference Angular velocity (rad/s)

% Initialize reference trajectory
xr = zeros(1, N);
yr = zeros(1, N);
theta_r = zeros(1, N);

% Initial conditions for reference
xr(1) = 0;
yr(1) = 0;
theta_r(1) = 0;

% Initialize robot measured states
xm = zeros(1, N); 
ym = zeros(1, N); 
theta_m = zeros(1, N);  

xm(1) = 0;
ym(1) = 0.5;   % Start along Y-axis (center of circle)
theta_m(1) = 0;

% Initialize dynamic model states
vm = zeros(1,N);    % actual linear velocity
wm = zeros(1,N);    % actual angular velocity

% Initialize vm_dot, wm_dot
vm_dot = zeros(1,N);
wm_dot = zeros(1,N);

% Control gains for Kinematic Controller
kx = 0.9;
ky = 0.9;
kth = 0.7;

% Save desired velocities
vd = zeros(1,N);
wd = zeros(1,N);

% ADRC parameters
kp = 13;
kd = 1;
w0 = 6;
bo = 12;

% Initialize ESO states (for both linear and angular)
x1_v = 0;    % ESO state 1 for linear velocity
x2_v = 0;    % ESO state 2 for linear velocity (disturbance)

x1_w = 0;    % ESO state 1 for angular velocity
x2_w = 0;    % ESO state 2 for angular velocity (disturbance)
vdd=0;
wdd=0;
dv = 0;
dw =0 ;

%% Parameters for dynamic model
k1 = 0.24089;
k2 = 0.2424;
k3 = -0.00093603;
k4 = 0.99629;
k5 = -0.0057256;
k6 = 1;

%% MAIN LOOP
for k = 2:N
    % Circle Change From 
    if k==160
    Wr=0.4;
    end
    %% -------- Generate Reference Trajectory --------
    theta = theta_r(k-1);      % current reference orientation
    
    dx = cos(theta) * Vr * T;    % delta x
    dy = sin(theta) * Vr * T;    % delta y
    d_theta = Wr * T;            % delta theta
    
    xr(k) = xr(k-1) + dx;        
    yr(k) = yr(k-1) + dy;        
    theta_r(k) = theta_r(k-1) + d_theta;   
    theta_r(k) = atan2(sin(theta_r(k)), cos(theta_r(k)));
    %% -------- Kinematic Controller to compute vd, wd --------
    % Tracking error in robot frame
    ex = ( (xr(k) - xm(k-1)) * cos(theta_m(k-1)) ) + ( (yr(k) - ym(k-1)) * sin(theta_m(k-1)) );
    ey = -( (xr(k) - xm(k-1)) * sin(theta_m(k-1)) ) + ( (yr(k) - ym(k-1)) * cos(theta_m(k-1)) );
    etheta = theta_r(k) - theta_m(k-1);
    
    % Normalize etheta
    etheta = atan2(sin(etheta), cos(etheta));
    
    % Compute desired linear and angular velocities
    vd(k) = (kx * ex) + (Vr * cos(etheta));
    wd(k) = (ky * Vr * ey) + Wr + (kth * sin(etheta));
    
    %% -------- ADRC for Linear Velocity Control (vd -> vm) --------
    e_v = vd(k) - vm(k-1);
    
    if k == 2
        e_v_prev = e_v;   % for derivative term
    end
    
    edot_v = e_v - e_v_prev;     % derivative of error
    u0_v = kp * e_v + kd * edot_v;      % PD control
    
    % ESO Update (Linear velocity)
    e_eso_v = x1_v - vm(k-1);
    dx1_v = x2_v + (bo * vdd) - (2*w0)*e_eso_v;    % u=0 temporary
    dx2_v = -(w0^2) * e_eso_v;
    
    x1_v = x1_v + dx1_v * T;
    x2_v = x2_v + dx2_v * T;
    
    % Final control input for dynamic model
    vdd = 0.083 * (u0_v - x2_v);
    
    e_v_prev = e_v;   % update for next iteration
    
    %% -------- ADRC for Angular Velocity Control (wd -> wm) --------
    e_w = wd(k) - wm(k-1);
    
    if k == 2
        e_w_prev = e_w;
    end
    
    edot_w = e_w - e_w_prev;
    u0_w = kp * e_w + kd * edot_w;
    
    % ESO Update (Angular velocity)
    e_eso_w = x1_w - wm(k-1);
    dx1_w = x2_w + (bo * wdd) - (2*w0)*e_eso_w;
    dx2_w = -(w0^2) * e_eso_w;
    
    x1_w = x1_w + dx1_w * T;
    x2_w = x2_w + dx2_w * T;
    
    % Final control input for dynamic model
    wdd = 0.083 * (u0_w - x2_w);
    
    e_w_prev = e_w;
    
    %% -------- Dynamic model --------
    vc= vdd;
    wc=wdd;
    % % Uncomment this if you want to apply a disturbance after 130 steps
    
     if k>=80 
         vc = vc + 0.7;   
         wc = wc + 0.7;
     end

     if k>=200 & k<=250
         dv = 2 * sin(0.3*pi*k*T) + 2 * cos(0.3*pi*k*T);
         dw = 2 * sin(0.3*pi*k*T) + 2 * sin(0.3*pi*k*T);
     end
    
    vm_dot(k) = ( (k3/k1) * (wm(k-1)^2) ) - ( (k4/k1) * vm(k-1) ) + (vc/k1) + dv;
    wm_dot(k) = - ( (k5/k2) * (vm(k-1)*wm(k-1)) ) - ( (k6/k2) * wm(k-1) ) + (wc/k2) + dw;
    
    % Integrate to update vm and wm
    vm(k) = vm(k-1) + vm_dot(k) * T;
    wm(k) = wm(k-1) + wm_dot(k) * T;
    
    %% -------- Update robot position --------
    % Robot motion
    dxm = vm(k) * cos(theta_m(k-1)) * T;
    dym = vm(k) * sin(theta_m(k-1)) * T;
    
    xm(k) = xm(k-1) + dxm;
    ym(k) = ym(k-1) + dym;
    theta_m(k) = theta_m(k-1) + wm(k) * T;
    
    % Normalize theta_m
    theta_m(k) = atan2(sin(theta_m(k)), cos(theta_m(k)));
end

%% -------- Plot results --------

% 1. Desired vs Actual Trajectory
figure;
plot(xr, yr, 'b--', 'LineWidth', 2); hold on;
plot(xm, ym, 'r-', 'LineWidth', 2);
xlabel('X position (m)');
ylabel('Y position (m)');
title('ADRC');
legend('Reference Trajectory','Actual Trajectory');
axis equal;
grid on;

% 2. Desired vs Actual Linear Velocity
figure;
plot((0:N-1)*T, vd, 'b--', 'LineWidth', 2); hold on;
plot((0:N-1)*T, vm, 'r-', 'LineWidth', 2);
ylabel('Linear Velocity (m/s)');
xlabel('Time (s)');
title('Desired vs Actual Linear Velocity');
legend('vd','vm');
grid on;

% 3. Desired vs Actual Angular Velocity
figure;
plot((0:N-1)*T, wd, 'b--', 'LineWidth', 2); hold on;
plot((0:N-1)*T, wm, 'r-', 'LineWidth', 2);
ylabel('Angular Velocity (rad/s)');
xlabel('Time (s)');
title('Desired vs Actual Angular Velocity');
legend('wd','wm');
grid on;

% 4. Desired vs Actual Theta (Orientation)
figure;
plot((0:N-1)*T, theta_r, 'b--', 'LineWidth', 2); hold on;
plot((0:N-1)*T, theta_m, 'r-', 'LineWidth', 2);
ylabel('Orientation \theta (rad)');
xlabel('Time (s)');
title('Desired vs Actual Orientation \theta');
legend('\theta_{ref}','\theta_{real}');
grid on;

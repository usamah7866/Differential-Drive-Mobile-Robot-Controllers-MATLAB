%% Starting control
clear
clc
N = 170;                 % Number of Iterations
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
ym(1) = 0.5;   % along Y-axis
theta_m(1) = 0;

% Initialize dynamic model states
vm = zeros(1,N);    % actual linear velocity
wm = zeros(1,N);    % actual angular velocity

% Initialize vm_dot, wm_dot
vm_dot = zeros(1,N);
wm_dot = zeros(1,N);

% Control gains
kx = 0.8;
ky = 0.8;
kth = 0.8;

% Save desired velocities
vd = zeros(1,N);
wd = zeros(1,N);

%% Parameters for dynamic model
k1 = 0.24089;
k2 = 0.2424;
k3 = -0.00093603;
k4 = 0.99629;
k5 = -0.0057256;
k6 = 1;

%% MAIN LOOP
for k = 2:N
    %% -------- Generate Reference Trajectory --------
    theta = theta_r(k-1);      % current reference orientation
    
    dx = cos(theta) * Vr * T;    % delta x
    dy = sin(theta) * Vr * T;    % delta y
    d_theta = Wr * T;            % delta theta
    
    xr(k) = xr(k-1) + dx;        
    yr(k) = yr(k-1) + dy;        
    theta_r(k) = theta_r(k-1) + d_theta;   
    
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
    
    %% -------- Dynamic model --------
    vc=vd(k);
    wc=wd(k);
     % Applying disturbance
    if k>100
        vc=0.5;   % disturbance value for Linear velocity
        wc=0.5;   % disturbance value for Angular velocity
    end
    vm_dot(k) = ( (k3/k1) * (wm(k-1)^2) ) - ( (k4/k1) * vm(k-1) ) + (vc/k1);
    wm_dot(k) = - ( (k5/k2) * (vm(k-1)*wm(k-1)) ) - ( (k6/k2) * wm(k-1) ) + (wc/k2);
    
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
title('Reference vs Actual Trajectory');
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

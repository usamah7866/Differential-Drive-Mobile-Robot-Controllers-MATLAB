%% Starting control with ADRC dynamic controller
clear
clc
N = 200;                 % Number of Iterations
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

% Control gains for Kinematic Controller & fuzzy T1
kx = 0.9;
ky = 0.9;
kth = 0.8;

kev=2;
kcev=0.1;
kuv=6;

kew=1.9;
kcew=0.1;
kuw=6;

% Save desired velocities
vd = zeros(1,N);
wd = zeros(1,N);

% ESO parameters
w0 = 6;
bo = 12;

% Initialize ESO states (for both linear and angular)
x1_v = 0;    % ESO state 1 for linear velocity
x2_v = 0;    % ESO state 2 for linear velocity (disturbance)

x1_w = 0;    % ESO state 1 for angular velocity
x2_w = 0;    % ESO state 2 for angular velocity (disturbance)

vdd=0;
wdd=0;
dv =0;
dw=0;
e_v_prev = 0;
e_w_prev = 0;


%%TSK FUZZY
% Rule 1: NE with NCE
c11=-0.5; s11u=0.25; s11l=0.15;
c21=-0.5; s21u=0.25; s21l=0.15;
% Rule 2: NE with ZCE
c12=-0.5; s12u=0.25; s12l=0.15;
c22=0; s22u=0.25; s22l=0.15;
% Rule 3: NE with PCE
c13=-0.5; s13u=0.25; s13l=0.15;
c23=0.5; s23u=0.25; s23l=0.15;

% Rule 4: ZE with NCE
c14=0; s14u=0.25; s14l=0.15;
c24=-0.5; s24u=0.25; s24l=0.15;
% Rule 5: ZE with ZCE
c15=0; s15u=0.25; s15l=0.15;
c25=0; s25u=0.25; s25l=0.15;
% Rule 6: ZE with PCE
c16=0; s16u=0.25; s16l=0.15;
c26=0.5; s26u=0.25; s26l=0.15;

% Rule 7: PE with NCE
c17=0.5; s17u=0.25; s17l=0.15;
c27=-0.5; s27u=0.25; s27l=0.15;
% Rule 8: PE with ZCE
c18=0.5; s18u=0.25; s18l=0.15;
c28=0; s28u=0.25; s28l=0.15;
% Rule 9: PE with PCE
c19=0.5; s19u=0.25; s19l=0.15;
c29=0.5; s29u=0.25; s29l=0.15;

% Parmeters of Rule Consequents (You can chose theses values between [0, 1]
a11=0.1; a21=0.9;
a12=1.5; a22=0.01; % changed a15 from 1 to 1.5  for step dis output
a13=0.5; a23=0.5;
a14=0.5; a24=1;
a15=0.9; a25=0.9; % changed a15 from 0.3 to 0.9  for step dis output
a16=0.2; a26=1;
a17=0.9; a27=0.9;
a18=1; a28=0;
a19=0.8; a29=0.2;


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
    % if k==160
    % Wr=0.4;
    % end
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
    EX(k)=xr(k)- xm(k-1);
    EY(k)=yr(k) - ym(k-1);
    ETH(k)=theta_r(k) - theta_m(k-1);

    EX2(k) = ex^2;
    EY2(k) = ey^2;
    % Normalize etheta
    etheta = atan2(sin(etheta), cos(etheta));
    
    % Compute desired linear and angular velocities
    vd(k) = (kx * ex) + (Vr * cos(etheta));
    wd(k) = (ky * Vr * ey) + Wr + (kth * sin(etheta));
    
     %% =============== FLC for Linear ==================================
    ev=vd(k)-vm(k-1);
    cev= ev-e_v_prev;

    e_v= kev*ev;
    ce_v= kcev*cev;

    % Fuzzification and Rules
% Upper Rules
r1u=(exp(-0.5*((e_v-c11)/s11u)^2))*(exp(-0.5*((ce_v-c21)/s21u)^2));
r2u=(exp(-0.5*((e_v-c12)/s12u)^2))*(exp(-0.5*((ce_v-c22)/s22u)^2));
r3u=(exp(-0.5*((e_v-c13)/s13u)^2))*(exp(-0.5*((ce_v-c23)/s23u)^2));
r4u=(exp(-0.5*((e_v-c14)/s14u)^2))*(exp(-0.5*((ce_v-c24)/s24u)^2));
r5u=(exp(-0.5*((e_v-c15)/s15u)^2))*(exp(-0.5*((ce_v-c25)/s25u)^2));
r6u=(exp(-0.5*((e_v-c16)/s16u)^2))*(exp(-0.5*((ce_v-c26)/s26u)^2));
r7u=(exp(-0.5*((e_v-c17)/s17u)^2))*(exp(-0.5*((ce_v-c27)/s27u)^2));
r8u=(exp(-0.5*((e_v-c18)/s18u)^2))*(exp(-0.5*((ce_v-c28)/s28u)^2));
r9u=(exp(-0.5*((e_v-c19)/s19u)^2))*(exp(-0.5*((ce_v-c29)/s29u)^2));
% Lower Rules
r1l=(exp(-0.5*((e_v-c11)/s11l)^2))*(exp(-0.5*((ce_v-c21)/s21l)^2));
r2l=(exp(-0.5*((e_v-c12)/s12l)^2))*(exp(-0.5*((ce_v-c22)/s22l)^2));
r3l=(exp(-0.5*((e_v-c13)/s13l)^2))*(exp(-0.5*((ce_v-c23)/s23l)^2));
r4l=(exp(-0.5*((e_v-c14)/s14l)^2))*(exp(-0.5*((ce_v-c24)/s24l)^2));
r5l=(exp(-0.5*((e_v-c15)/s15l)^2))*(exp(-0.5*((ce_v-c25)/s25l)^2));
r6l=(exp(-0.5*((e_v-c16)/s16l)^2))*(exp(-0.5*((ce_v-c26)/s26l)^2));
r7l=(exp(-0.5*((e_v-c17)/s17l)^2))*(exp(-0.5*((ce_v-c27)/s27l)^2));
r8l=(exp(-0.5*((e_v-c18)/s18l)^2))*(exp(-0.5*((ce_v-c28)/s28l)^2));
r9l=(exp(-0.5*((e_v-c19)/s19l)^2))*(exp(-0.5*((ce_v-c29)/s29l)^2));
% Defuzzification
du1=a11*e_v+a21*ce_v;
du2=a12*e_v+a22*ce_v;
du3=a13*e_v+a23*ce_v;
du4=a14*e_v+a24*ce_v;
du5=a15*e_v+a25*ce_v;
du6=a16*e_v+a26*ce_v;
du7=a17*e_v+a27*ce_v;
du8=a18*e_v+a28*ce_v;
du9=a19*e_v+a29*ce_v;
denU=r1u+r2u+r3u+r4u+r5u+r6u+r7u+r8u+r9u;
denL=r1l+r2l+r3l+r4l+r5l+r6l+r7l+r8l+r9l;
numU=du1*r1u+du2*r2u+du3*r3u+du4*r4u+du5*r5u+du6*r6u+du7*r7u+du8*r8u+du9*r9u;
numL=du1*r1l+du2*r2l+du3*r3l+du4*r4l+du5*r5l+du6*r6l+du7*r7l+du8*r8l+du9*r9l;

if denU == 0
    dUV=0;
else
    dUV=numU/denU;  
end
if denL == 0
    dLV=0;
else
    dLV=numL/denL;  
end

u0_v=kuv*(dUV+dLV)*0.5;

   %% -------- ADRC for Linear Velocity Control --------
    
    % ESO Update (Linear velocity)
    e_eso_v = x1_v - vm(k-1);
    dx1_v = x2_v + (bo * vdd) - (2*w0)*e_eso_v;    % u=0 temporary
    dx2_v = -(w0^2) * e_eso_v;
    
    x1_v = x1_v + dx1_v * T;
    x2_v = x2_v + dx2_v * T;
    
    % Final control input for dynamic model
    vdd = 0.083 * (u0_v - x2_v);

    e_v_prev = ev;   % update for next iteration
    
     %% =============== FLC for Angular ==================================
    ew=wd(k)-wm(k-1);
    cew=ew-e_w_prev;
    
    e_w= kew*ew;
    ce_w= kcew*cew;
    
       % Fuzzification and Rules
% Upper Rules
r1u=(exp(-0.5*((e_w-c11)/s11u)^2))*(exp(-0.5*((ce_w-c21)/s21u)^2));
r2u=(exp(-0.5*((e_w-c12)/s12u)^2))*(exp(-0.5*((ce_w-c22)/s22u)^2));
r3u=(exp(-0.5*((e_w-c13)/s13u)^2))*(exp(-0.5*((ce_w-c23)/s23u)^2));
r4u=(exp(-0.5*((e_w-c14)/s14u)^2))*(exp(-0.5*((ce_w-c24)/s24u)^2));
r5u=(exp(-0.5*((e_w-c15)/s15u)^2))*(exp(-0.5*((ce_w-c25)/s25u)^2));
r6u=(exp(-0.5*((e_w-c16)/s16u)^2))*(exp(-0.5*((ce_w-c26)/s26u)^2));
r7u=(exp(-0.5*((e_w-c17)/s17u)^2))*(exp(-0.5*((ce_w-c27)/s27u)^2));
r8u=(exp(-0.5*((e_w-c18)/s18u)^2))*(exp(-0.5*((ce_w-c28)/s28u)^2));
r9u=(exp(-0.5*((e_w-c19)/s19u)^2))*(exp(-0.5*((ce_w-c29)/s29u)^2));
% Lower Rules
r1l=(exp(-0.5*((e_w-c11)/s11l)^2))*(exp(-0.5*((ce_w-c21)/s21l)^2));
r2l=(exp(-0.5*((e_w-c12)/s12l)^2))*(exp(-0.5*((ce_w-c22)/s22l)^2));
r3l=(exp(-0.5*((e_w-c13)/s13l)^2))*(exp(-0.5*((ce_w-c23)/s23l)^2));
r4l=(exp(-0.5*((e_w-c14)/s14l)^2))*(exp(-0.5*((ce_w-c24)/s24l)^2));
r5l=(exp(-0.5*((e_w-c15)/s15l)^2))*(exp(-0.5*((ce_w-c25)/s25l)^2));
r6l=(exp(-0.5*((e_w-c16)/s16l)^2))*(exp(-0.5*((ce_w-c26)/s26l)^2));
r7l=(exp(-0.5*((e_w-c17)/s17l)^2))*(exp(-0.5*((ce_w-c27)/s27l)^2));
r8l=(exp(-0.5*((e_w-c18)/s18l)^2))*(exp(-0.5*((ce_w-c28)/s28l)^2));
r9l=(exp(-0.5*((e_w-c19)/s19l)^2))*(exp(-0.5*((ce_w-c29)/s29l)^2));

% Defuzzification
du1=a11*e_w+a21*ce_w;
du2=a12*e_w+a22*ce_w;
du3=a13*e_w+a23*ce_w;
du4=a14*e_w+a24*ce_w;
du5=a15*e_w+a25*ce_w;
du6=a16*e_w+a26*ce_w;
du7=a17*e_w+a27*ce_w;
du8=a18*e_w+a28*ce_w;
du9=a19*e_w+a29*ce_w;
denU=r1u+r2u+r3u+r4u+r5u+r6u+r7u+r8u+r9u;
denL=r1l+r2l+r3l+r4l+r5l+r6l+r7l+r8l+r9l;
numU=du1*r1u+du2*r2u+du3*r3u+du4*r4u+du5*r5u+du6*r6u+du7*r7u+du8*r8u+du9*r9u;
numL=du1*r1l+du2*r2l+du3*r3l+du4*r4l+du5*r5l+du6*r6l+du7*r7l+du8*r8l+du9*r9l;

if denU == 0
    dUW=0;
else
    dUW=numU/denU;  
end

if denL == 0
    dLW=0;
else
    dLW=numL/denL;  
end
u0_w=kuw*(dUW+dLW)*0.5;

  %% -------- ADRC for Angular Velocity Control (wd -> wm) --------
    
    % ESO Update (Angular velocity)
    e_eso_w = x1_w - wm(k-1);
    dx1_w = x2_w + (bo * wdd) - (2*w0)*e_eso_w;
    dx2_w = -(w0^2) * e_eso_w;
    
    x1_w = x1_w + dx1_w * T;
    x2_w = x2_w + dx2_w * T;
    
    % Final control input for dynamic model
    wdd = 0.083 * (u0_w - x2_w);
    
    e_w_prev = ew;
    
    %% -------- Dynamic model --------
    vc= vdd;
    wc=wdd;
     % if k>=80
     %     vc = vc + 0.9;   
     %     wc = wc + 0.9;
     % end
      if k>=80
          dv = 3.9;
          dw = 3.9;
     end
     % if k>=110 && k<=190
     %      vc = vc + 0.6 * sin(0.3*pi*k*T) + 0.6 * cos(0.3*pi*k*T);   
     %      wc = wc + 0.6 * sin(0.3*pi*k*T) + 0.6 * sin(0.3*pi*k*T);
     %  end
     % 
     % if k>=110 && k<=190
     %     dv = 2.5 * sin(0.3*pi*k*T) + 2.5 * cos(0.3*pi*k*T);
     %     dw = 2.5 * sin(0.3*pi*k*T) + 2.5 * sin(0.3*pi*k*T);
     % end



    
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
    MAEALL(k)=sum(abs(EX)+abs(EY))/k;
end

%% -------- Plot results --------
%% -------- Plot results -------

outputFolder = 'figures';  % You can change this to any directory you like

% Create the folder if it doesn't exist
if ~exist(outputFolder, 'dir')
    mkdir(outputFolder);
end


% 1. Desired vs Actual Trajectory
figure;
plot(xr, yr, 'b--', 'LineWidth', 1.5); hold on;
plot(xm, ym, 'K-', 'LineWidth', 1.5);
xlabel('X position (m)');
ylabel('Y position (m)');
legend('Reference Trajectory','ADRC-IT2TSKFPD');
axis equal;
grid on;
savefig(fullfile(outputFolder, 'traj_TSK.fig'));

% 2. Desired vs Actual Linear Velocity
figure;
plot((0:N-1)*T, vd, 'b--', 'LineWidth', 2); hold on;
plot((0:N-1)*T, vm, 'k-', 'LineWidth', 2);
ylabel('Linear Velocity (m/s)');
xlabel('Time (s)');
title('Desired vs Actual Linear Velocity');
legend('vd','vm');
grid on;
savefig(fullfile(outputFolder, 'LinearVelocity_tsk.fig'));

% 3. Desired vs Actual Angular Velocity
figure;
plot((0:N-1)*T, wd, 'b--', 'LineWidth', 2); hold on;
plot((0:N-1)*T, wm, 'k-', 'LineWidth', 2);
ylabel('Angular Velocity (rad/s)');
xlabel('Time (s)');
title('Desired vs Actual Angular Velocity');
legend('wd','wm');
grid on;
savefig(fullfile(outputFolder, 'AngularVelocity_tsk.fig'));


% 6. X-axis Position: Reference vs Actual
figure;
plot((0:N-1)*T, xr, 'b--', 'LineWidth', 2); hold on;
plot((0:N-1)*T, xm, 'k-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('X Position (m)');
title('X-axis: Reference vs Actual');
legend('X_{ref}', 'ADRC-T1FPD');
grid on;
%savefig(fullfile(outputFolder, 'X_Position_t1.fig'));
% 7. Y-axis Position: Reference vs Actual
figure;
plot((0:N-1)*T, yr, 'b--', 'LineWidth', 2); hold on;
plot((0:N-1)*T, ym, 'k-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Y Position (m)');
title('Y-axis: Reference vs Actual');
legend('Y_{ref}', 'ADRC-IT2FPD');
grid on;
%savefig(fullfile(outputFolder, 'Y_Position_t1.fig'));
% 4. Desired vs Actual Theta (Orientation)
figure;
plot((0:N-1)*T, theta_r, 'b--', 'LineWidth', 2); hold on;
plot((0:N-1)*T, theta_m, 'k-', 'LineWidth', 2);
ylabel('Orientation \theta (rad)');
xlabel('Time (s)');
title('Desired vs Actual Orientation \theta');
legend('\theta_{ref}','Proposed Controller');
grid on;
%savefig(fullfile(outputFolder, 'ThetaOrientation_t1.fig'));

% 5. MAE Error Over Time

k=2:N;
t=0.1*k;
figure;
plot(t, MAEALL(k), 'k-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Absolute Position Error (m)');
legend('ADRC-IT2TSKFPD');
grid on;
% savefig(fullfile(outputFolder, 'MAE_PositionError_tsk.fig'));


% MAE and RMSE
% MAE and RMSE
RMSE  = sqrt(sum((EX2)+ (EY2))/N);
MAE = MAEALL(N);  % Overall MAE (optional average over dimensions)

% Display results
fprintf('--- Error Metrics ---\n');
fprintf('RMSE = %.4f m\n', RMSE);
fprintf('MAE  = %.4f m\n', MAE);
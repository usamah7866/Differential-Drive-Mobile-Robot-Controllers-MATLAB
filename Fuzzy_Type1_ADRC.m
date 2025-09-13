%% Starting control with ADRC dynamic controller
clear
clc
N = 400;                 % Number of Iterations
T = 0.1;                 % Sampling time (sec.)

Vr = 0.25;               % Reference Linear velocity (m/s)
Wr = 0.5;                % Reference Angular velocity (rad/s)

x1_v_all = zeros(1, N);
x2_v_all = zeros(1, N);
x1_w_all = zeros(1, N);
x2_w_all = zeros(1, N);


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
kuv=10;

kew=1.9;
kcew=0.1;
kuw=10;


% Save desired velocities
vd = zeros(1,N);
wd = zeros(1,N);

% ESO parameters
w0 = 6.4;
bo = 12;

% Initialize ESO states (for both linear and angular)
x1_v = 0;    % ESO state 1 for linear velocity
x2_v = 0;    % ESO state 2 for linear velocity (disturbance)

x1_w = 0;    % ESO state 1 for angular velocity
x2_w = 0;    % ESO state 2 for angular velocity (disturbance)
vdd=0;
wdd=0;
dv = 0;
dw = 0;
Mae=0;


% paramters of MF for input E
%MF1 NL
a11=-1; b11=-0.66; c11=-0.33;
%MF2 NS
a12=-0.66; b12=-0.33; c12=0;
%MF3 Z
a13=-0.33; b13=0; c13=0.33;
%MF4 PS
a14=0; b14=0.33; c14=0.66;
%MF5 PL
a15=0.33; b15=0.66; c15=1;
%==========================
% paramters of MF for input CE
%MF1 NL
a21=-1; b21=-0.66; c21=-0.33;
%MF2 NS
a22=-0.66; b22=-0.33; c22=0;
%MF3 Z
a23=-0.33; b23=0; c23=0.33;
%MF4 PS
a24=0; b24=0.33; c24=0.66;
%MF5 PL
a25=0.33; b25=0.66; c25=1;
%=================
e_v_prev = 0;
e_w_prev = 0;

%% Parameters for dynamic model
k1 = 0.24089;
k2 = 0.2424;
k3 = -0.00093603;
k4 = 0.99629;
k5 = -0.0057256;
k6 = 1;
tic;
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
    e_v=vd(k)-vm(k-1);
    ce_v=e_v-e_v_prev;

    NLE=0; NSE=0; ZE=0; PSE=0; PLE=0;
    NLCE=0; NSCE=0; ZCE=0; PSCE=0; PLCE=0;
    % input gains
    e=kev*e_v;
    ce=kcev*ce_v;
    %===============================
    %fuzzification of E
%MF1: NLE
if e>=a11 & e<=b11
    NLE=(a11-e)/(a11-b11);
elseif e>=b11 & e<=c11
    NLE=(c11-e)/(c11-b11);
else
    NLE=0;
end
%================
%MF2: NSE
if e>=a12 & e<=b12
    NSE=(a12-e)/(a12-b12);
elseif e>=b12 & e<=c12
    NSE=(c12-e)/(c12-b12);
else
    NSE=0;
end
%===============
 %MF3: ZE
if e>=a13 & e<=b13
    ZE=(a13-e)/(a13-b13);
elseif e>=b13 & e<=c13
    ZE=(c13-e)/(c13-b13);
else
    ZE=0;
end 
%===============
 %MF4: PSE
if e>=a14 & e<=b14
    PSE=(a14-e)/(a14-b14);
elseif e>=b14 & e<=c14
    PSE=(c14-e)/(c14-b14);
else
    PSE=0;
end 
%===============
 %MF5: PLE
if e>=a15 & e<=b15
    PLE=(a15-e)/(a15-b15);
elseif e>=b15 & e<=c15 
    PLE=(c15-e)/(c15-b15);
else
    PLE=0;
end 
%========================================
 %fuzzification of CE
%MF1: NLCE
if ce>=a21 & ce<=b21
    NLCE=(a21-ce)/(a21-b21);
elseif ce>=b21 & ce<=c21
    NLCE=(c21-ce)/(c21-b21);
else
    NLCE=0;
end
%================
%MF2: NSCE
if ce>=a22 & ce<=b22
    NSCE=(a22-ce)/(a22-b22);
elseif ce>=b22 & ce<=c22
    NSCE=(c22-ce)/(c22-b22);
else
    NSCE=0;
end
%===============
 %MF3: ZCE
if ce>=a23 & ce<=b23
    ZCE=(a23-ce)/(a23-b23);
elseif ce>=b23 & ce<=c23
    ZCE=(c23-ce)/(c23-b23);
else
    ZCE=0;
end 
%===============
 %MF4: PSCE
if ce>=a24 & ce<=b24
    PSCE=(a24-ce)/(a24-b24);
elseif ce>=b24 & ce<=c24
    PSCE=(c24-ce)/(c24-b24);
else
    PSCE=0;
end 
%===============
 %MF5: PLCE
if ce>=a25 & ce<=b25
    PLCE=(a25-ce)/(a25-b25);
elseif ce>=b25 & ce<=c25
    PLCE=(c25-ce)/(c25-b25);
else
    PLCE=0;
end 
%========================================    
% Rules
r1=min(PLE,NLCE);
r2=min(PLE,NSCE);
r3=min(PLE,ZCE);
r4=min(PLE,PSCE);
r5=min(PLE,PLCE);
r6=min(PSE,NLCE);
r7=min(PSE,NSCE);
r8=min(PSE,ZCE);
r9=min(PSE,PSCE);
r10=min(PSE,PLCE);
r11=min(ZE,NLCE);
r12=min(ZE,NSCE);
r13=min(ZE,ZCE);
r14=min(ZE,PSCE);
r15=min(ZE,PLCE);
r16=min(NSE,NLCE);
r17=min(NSE,NSCE);
r18=min(NSE,ZCE);
r19=min(NSE,PSCE);
r20=min(NSE,PLCE);
r21=min(NLE,NLCE);
r22=min(NLE,NSCE);
r23=min(NLE,ZCE);
r24=min(NLE,PSCE);
r25=min(NLE,PLCE);

h1=max(max(r3,r4),r5);     h2=max(max(r9,r10),r15);
PL=max(h1,h2);
%=========
h3=max(max(r2,r8),max(r14,r20));
PS=h3;
%========
h4=max(max(r1,r7),r13);    h5=max(r19,r25);
Z=max(h4,h5);
%=========
h6=max(max(r6,r12),max(r18,r24));
NS=h6;
%=========
h7=max(max(r11,r16),r17);   h8=max(max(r21,r22),r23);
NL=max(h7,h8);
%=======================================
% defuzzification
num=(NL*(-0.601))+(NS*(-0.2411))+(Z*0)+(PS*0.33)+(PL*0.66);
den=NL+NS+Z+PS+PL;
if den == 0
    cuvn=0;
else
    cuvn=num/den;  
end
%=============
%output gain
u0_v =kuv *cuvn;
%=========================================================================

    %% -------- ADRC for Linear Velocity Control --------
    
    % ESO Update (Linear velocity)
    e_eso_v = x1_v - vm(k-1);
    dx1_v = x2_v + (bo * vdd) - (2*w0)*e_eso_v;    % u=0 temporary
    dx2_v = -(w0^2) * e_eso_v;
    
    x1_v = x1_v + dx1_v * T;
    x2_v = x2_v + dx2_v * T;
    
    % Final control input for dynamic model
    vdd = 0.083 * (u0_v - x2_v);
    
    e_v_prev = e_v;   % update for next iteration
    
     %% =============== FLC for Angular ==================================
    e_w=wd(k)-wm(k-1);
    ce_w=e_w-e_w_prev;
    
    NLE=0; NSE=0; ZE=0; PSE=0; PLE=0;
    NLCE=0; NSCE=0; ZCE=0; PSCE=0; PLCE=0;
    % input gains

    e=kew*e_w;
    ce=kcew*ce_w;
    %===============================
    %fuzzification of E
%MF1: NLE
if e>=a11 & e<=b11
    NLE=(a11-e)/(a11-b11);
elseif e>=b11 & e<=c11
    NLE=(c11-e)/(c11-b11);
else
    NLE=0;
end
%================
%MF2: NSE
if e>=a12 & e<=b12
    NSE=(a12-e)/(a12-b12);
elseif e>=b12 & e<=c12
    NSE=(c12-e)/(c12-b12);
else
    NSE=0;
end
%===============
 %MF3: ZE
if e>=a13 & e<=b13
    ZE=(a13-e)/(a13-b13);
elseif e>=b13 & e<=c13
    ZE=(c13-e)/(c13-b13);
else
    ZE=0;
end 
%===============
 %MF4: PSE
if e>=a14 & e<=b14
    PSE=(a14-e)/(a14-b14);
elseif e>=b14 & e<=c14
    PSE=(c14-e)/(c14-b14);
else
    PSE=0;
end 
%===============
 %MF5: PLE
if e>=a15 & e<=b15
    PLE=(a15-e)/(a15-b15);
elseif e>=b15 & e<=c15 
    PLE=(c15-e)/(c15-b15);
else
    PLE=0;
end 
%========================================
 %fuzzification of CE
%MF1: NLCE
if ce>=a21 & ce<=b21
    NLCE=(a21-ce)/(a21-b21);
elseif ce>=b21 & ce<=c21
    NLCE=(c21-ce)/(c21-b21);
else
    NLCE=0;
end
%================
%MF2: NSCE
if ce>=a22 & ce<=b22
    NSCE=(a22-ce)/(a22-b22);
elseif ce>=b22 & ce<=c22
    NSCE=(c22-ce)/(c22-b22);
else
    NSCE=0;
end
%===============
 %MF3: ZCE
if ce>=a23 & ce<=b23
    ZCE=(a23-ce)/(a23-b23);
elseif ce>=b23 & ce<=c23
    ZCE=(c23-ce)/(c23-b23);
else
    ZCE=0;
end 
%===============
 %MF4: PSCE
if ce>=a24 & ce<=b24
    PSCE=(a24-ce)/(a24-b24);
elseif ce>=b24 & ce<=c24
    PSCE=(c24-ce)/(c24-b24);
else
    PSCE=0;
end 
%===============
 %MF5: PLCE
if ce>=a25 & ce<=b25
    PLCE=(a25-ce)/(a25-b25);
elseif ce>=b25 & ce<=c25
    PLCE=(c25-ce)/(c25-b25);
else
    PLCE=0;
end 
%========================================    
% Rules
r1=min(PLE,NLCE);
r2=min(PLE,NSCE);
r3=min(PLE,ZCE);
r4=min(PLE,PSCE);
r5=min(PLE,PLCE);
r6=min(PSE,NLCE);
r7=min(PSE,NSCE);
r8=min(PSE,ZCE);
r9=min(PSE,PSCE);
r10=min(PSE,PLCE);
r11=min(ZE,NLCE);
r12=min(ZE,NSCE);
r13=min(ZE,ZCE);
r14=min(ZE,PSCE);
r15=min(ZE,PLCE);
r16=min(NSE,NLCE);
r17=min(NSE,NSCE);
r18=min(NSE,ZCE);
r19=min(NSE,PSCE);
r20=min(NSE,PLCE);
r21=min(NLE,NLCE);
r22=min(NLE,NSCE);
r23=min(NLE,ZCE);
r24=min(NLE,PSCE);
r25=min(NLE,PLCE);

h1=max(max(r3,r4),r5);     h2=max(max(r9,r10),r15);
PL=max(h1,h2);
%=========
h3=max(max(r2,r8),max(r14,r20));
PS=h3;
%========
h4=max(max(r1,r7),r13);    h5=max(r19,r25);
Z=max(h4,h5);
%=========
h6=max(max(r6,r12),max(r18,r24));
NS=h6;
%=========
h7=max(max(r11,r16),r17);   h8=max(max(r21,r22),r23);
NL=max(h7,h8);
%=======================================
% defuzzification
num=(NL*(-0.66))+(NS*(-0.33))+(Z*0)+(PS*0.33)+(PL*0.66);
den=NL+NS+Z+PS+PL;
if den == 0
    cuwn=0;
else
    cuwn=num/den;  
end
%=============
%output gain
u0_w=kuw*cuwn;
%======================================================================
    %% -------- ADRC for Angular Velocity Control (wd -> wm) --------
    
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
     if k>=80
         vc = vc + 0.99;   
         wc = wc + 0.99;
     end
     %  if k>=80
     %      dv = 3.9;
     %      dw = 3.9;
     % end
     if k>=110 && k<=190
          vc = vc + 0.6 * sin(0.3*pi*k*T) + 0.6 * cos(0.3*pi*k*T);   
          wc = wc + 0.6 * sin(0.3*pi*k*T) + 0.6 * sin(0.3*pi*k*T);
      end

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
    x1_v_all(k) = x1_v;
x2_v_all(k) = x2_v;
x1_w_all(k) = x1_w;
x2_w_all(k) = x2_w;

end
elapsedTime = toc;   % Stop timer and record elapsed time
fprintf('Total execution time: %.6f seconds\n', elapsedTime);
fprintf('Average time per iteration: %.6f seconds\n', elapsedTime/N);

%% -------- Plot results -------

outputFolder = 'figures';  % You can change this to any directory you like

% Create the folder if it doesn't exist
if ~exist(outputFolder, 'dir')
    mkdir(outputFolder);
end


% 1. Desired vs Actual Trajectory
figure;
plot(xr, yr, 'b--', 'LineWidth', 1.5); hold on;
plot(xm, ym, 'r-', 'LineWidth', 1.5);
xlabel('X position (m)');
ylabel('Y position (m)');
legend('Reference Trajectory','ADRC-T1FPD');
axis equal;
grid on;
savefig(fullfile(outputFolder, 'traj_T1FPD.fig'));


% figure;
% plot((0:N-1)*T, dv, 'b--', 'LineWidth', 2); 
% ylabel('dis');
% xlabel('Time (s)');
% title('dis');
% grid on;

% 2. Desired vs Actual Linear Velocity
figure;
plot((0:N-1)*T, vd, 'b--', 'LineWidth', 2); hold on;
plot((0:N-1)*T, vm, 'r-', 'LineWidth', 2);
ylabel('Linear Velocity (m/s)');
xlabel('Time (s)');
title('Desired vs Actual Linear Velocity');
legend('vd','vm');
grid on;
savefig(fullfile(outputFolder, 'LinearVelocity_T1FPD.fig'));
% 
% 3. Desired vs Actual Angular Velocity
figure;
plot((0:N-1)*T, wd, 'b--', 'LineWidth', 2); hold on;
plot((0:N-1)*T, wm, 'r-', 'LineWidth', 2);
ylabel('Angular Velocity (rad/s)');
xlabel('Time (s)');
title('Desired vs Actual Angular Velocity');
legend('wd','wm');
grid on;
savefig(fullfile(outputFolder, 'AngularVelocity_T1FPD.fig'));

% 6. X-axis Position: Reference vs Actual
figure;
plot((0:N-1)*T, xr, 'b--', 'LineWidth', 2); hold on;
plot((0:N-1)*T, xm, 'r-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('X Position (m)');
title('X-axis: Reference vs Actual');
legend('X_{ref}', 'ADRC-T1FPD');
grid on;
%savefig(fullfile(outputFolder, 'X_Position_t1.fig'));
% 7. Y-axis Position: Reference vs Actual
figure;
plot((0:N-1)*T, yr, 'b--', 'LineWidth', 2); hold on;
plot((0:N-1)*T, ym, 'r-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Y Position (m)');
title('Y-axis: Reference vs Actual');
legend('Y_{ref}', 'ADRC-IT2FPD');
grid on;
%savefig(fullfile(outputFolder, 'Y_Position_t1.fig'));
% 4. Desired vs Actual Theta (Orientation)
figure;
plot((0:N-1)*T, theta_r, 'b--', 'LineWidth', 2); hold on;
plot((0:N-1)*T, theta_m, 'r-', 'LineWidth', 2);
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
plot(t, MAEALL(k), 'r-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Absolute Position Error (m)');
legend('ADRC-T1FPD');
grid on;
% savefig(fullfile(outputFolder, 'MAE_PositionError_t1.fig'));
% 


% MAE and RMSE
RMSE  = sqrt(sum((EX2)+ (EY2))/N);
MAE = MAEALL(N);  % Overall MAE (optional average over dimensions)


% Display results
fprintf('--- Error Metrics ---\n');
fprintf('RMSE = %.4f m\n', RMSE);
fprintf('MAE  = %.4f m\n', MAE);


figure;
subplot(2,1,1);
plot((0:N-1)*T, x1_v_all, 'b', 'LineWidth', 1.5); hold on;
plot((0:N-1)*T, x2_v_all, 'r--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('ESO States for v');
legend('x1\_v (est. v)', 'x2\_v (est. disturbance)');
title('ADRC ESO States for Linear Velocity');

subplot(2,1,2);
plot((0:N-1)*T, x1_w_all, 'g', 'LineWidth', 1.5); hold on;
plot((0:N-1)*T, x2_w_all, 'm--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('ESO States for w');
legend('x1\_w (est. w)', 'x2\_w (est. disturbance)');
title('ADRC ESO States for Angular Velocity');

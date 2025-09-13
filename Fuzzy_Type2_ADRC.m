%% Starting control with ADRC dynamic controller
clear
clc
N = 400;                 % Number of Iterations
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

% Control gains for Kinematic Controller & FUZZY T2 
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
w0 = 6;
bo = 12;

% Initialize ESO states (for both linear and angular)
x1_v = 0;    % ESO state 1 for linear velocity
x2_v = 0;    % ESO state 2 for linear velocity (disturbance)

x1_w = 0;    % ESO state 1 for angular velocity
x2_w = 0;    % ESO state 2 for angular velocity (disturbance)
vdd=0;
wdd=0;
dv=0;
dw=0;
Mae2=0;
% paramters of MF for input E
%MF1 NL
m11u=0.1; m11l=0.1;
a11u=-1-m11u; b11u=-0.66; c11u=-0.33+m11u;                   a11l=-1+m11l; b11l=-0.66; c11l=-0.33-m11l;
%MF2 NS
m12u=0.1;  m12l=0.1;
a12u=-0.66-m12u; b12u=-0.33; c12u=0.0005;            a12l=-0.66+m12l; b12l=-0.33; c12l=0.0001;
%MF3 Z
m13u=0.1;  m13l=0.1;
a13u=-0.33-m13u; b13u=0; c13u=0.33+m13u;       a13l=-0.33+m13l; b13l=0; c13l=0.33-m13l;
%MF4 PS
m14u=0.1;   m14l=0.1;
a14u=0; b14u=0.33; c14u=0.66+m14u;              a14l=0; b14l=0.33; c14l=0.66-m14l;
%MF5 PL
m15u=0.1; m15l=0.1;
a15u=0.33-m15u; b15u=0.66;  c15u=1+m15u;                    a15l=0.33+m15l; b15l=0.66; c15l=1-m15l;
%==========================
% paramters of MF for input CE
%MF1 NL
m21u=0.1; m21l=0.1;
a21u=-1-m21u; b21u=-0.66; c21u=-0.33+m21u;                   a21l=-1+m21l; b21l=-0.66; c21l=-0.33-m21l;
%MF2 NS
m22u=0.1;  m22l=0.1;
a22u=-0.66-m22u; b22u=-0.33; c22u=0.0005;            a22l=-0.66+m22l; b22l=-0.33; c22l=0.0001;
%MF3 Z
m23u=0.1;  m23l=0.1;
a23u=-0.33-m23u; b23u=0; c23u=0.33+m23u;       a23l=-0.33+m23l; b23l=0; c23l=0.33-m23l;
%MF4 PS
m24u=0.1;   m24l=0.1;
a24u=0; b24u=0.33; c24u=0.66+m24u;              a24l=0; b24l=0.33; c24l=0.66-m24l;
%MF5 PL
m25u=0.1; m25l=0.1;
a25u=0.33-m25u; b25u=0.66;  c25u=1+m25u;                    a25l=0.33+m25l; b25l=0.66; c25l=1-m25l;
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
    EX(k)=ex;
    EY(k)=ey;
    ETH(k)=etheta;
    % Normalize etheta
    etheta = atan2(sin(etheta), cos(etheta));
    
    % Compute desired linear and angular velocities
    vd(k) = (kx * ex) + (Vr * cos(etheta));
    wd(k) = (ky * Vr * ey) + Wr + (kth * sin(etheta));
    
   %% =============== FLC for Linear ==================================
    e_v=vd(k)-vm(k-1);
    ce_v=e_v-e_v_prev;
    NLEU=0; NSEU=0; ZEU=0; PSEU=0; PLEU=0;       NLEL=0; NSEL=0; ZEL=0; PSEL=0; PLEL=0;
    NLCEU=0; NSCEU=0; ZCEU=0; PSCEU=0; PLCEU=0;       NLCEL=0; NSCEL=0; ZCEL=0; PSCEL=0; PLCEL=0;
    % input gains
    e=kev*e_v;
    ce=kcev*ce_v;
    %===============================
    %fuzzification of E
%MF1: NLEU
if e>=a11u & e<=b11u
    NLEU=(a11u-e)/(a11u-b11u);
elseif e>=b11u & e<=c11u
    NLEU=(c11u-e)/(c11u-b11u);
else
    NLEU=0;
end
%MF1: NLEL
if e>=a11l & e<=b11l
    NLEL=(a11l-e)/(a11l-b11l);
elseif e>=b11l & e<=c11l
    NLEL=(c11l-e)/(c11l-b11l);
else
    NLEL=0;
end
%================
%MF2: NSEU
if e>=a12u & e<=b12u
    NSEU=(a12u-e)/(a12u-b12u);
elseif e>=b12u & e<=c12u
    NSEU=(c12u-e)/(c12u-b12u);
else
    NSEU=0;
end
%MF2: NSEL
if e>=a12l & e<=b12l
    NSEL=(a12l-e)/(a12l-b12l);
elseif e>=b12l & e<=c12l
    NSEL=(c12l-e)/(c12l-b12l);
else
    NSEL=0;
end
%===============
 %MF3: ZEU
if e>=a13u & e<=b13u
    ZEU=(a13u-e)/(a13u-b13u);
elseif e>=b13u & e<=c13u
    ZEU=(c13u-e)/(c13u-b13u);
else
    ZEU=0;
end 
 %MF3: ZEL
if e>=a13l & e<=b13l
    ZEL=(a13l-e)/(a13l-b13l);
elseif e>=b13l & e<=c13l
    ZEL=(c13l-e)/(c13l-b13l);
else
    ZEL=0;
end 
%===============
 %MF4: PSEU
if e>=a14u & e<=b14u
    PSEU=(a14u-e)/(a14u-b14u);
elseif e>=b14u & e<=c14u
    PSEU=(c14u-e)/(c14u-b14u);
else
    PSEU=0;
end 
 %MF4: PSEL
if e>=a14u & e<=b14u
    PSEL=(a14u-e)/(a14u-b14u);
elseif e>=b14u & e<=c14u
    PSEL=(c14u-e)/(c14u-b14u);
else
    PSEL=0;
end 
%===============
 %MF5: PLEU
if e>=a15u & e<=b15u
    PLEU=(a15u-e)/(a15u-b15u);
elseif e>=b15u & e<=c15u 
    PLEU=(c15u-e)/(c15u-b15u);
else
    PLEU=0;
end 
%MF5: PLE
if e>=a15u & e<=b15u
    PLEL=(a15u-e)/(a15u-b15u);
elseif e>=b15u & e<=c15u
    PLEL=(c15u-e)/(c15u-b15u);
else
    PLEL=0;
end 
%========================================
 %fuzzification of CE
%MF1: NLCEU
if ce>=a21u & ce<=b21u
    NLCEU=(a21u-ce)/(a21u-b21u);
elseif ce>=b21u & ce<=c21u
    NLCEU=(c21u-ce)/(c21u-b21u);
else
    NLCEU=0;
end
%MF1: NLCEL
if ce>=a21l & ce<=b21l
    NLCEL=(a21l-ce)/(a21l-b21l);
elseif ce>=b21l & ce<=c21l
    NLCEL=(c21l-ce)/(c21l-b21l);
else
    NLCEL=0;
end
%================
%MF2: NSCEU
if ce>=a22u & ce<=b22u
    NSCEU=(a22u-ce)/(a22u-b22u);
elseif ce>=b22u & ce<=c22u
    NSCEU=(c22u-ce)/(c22u-b22u);
else
    NSCEU=0;
end
%MF2: NSCEL
if ce>=a22l & ce<=b22l
    NSCEL=(a22l-ce)/(a22l-b22l);
elseif ce>=b22l & ce<=c22l
    NSCEL=(c22l-ce)/(c22l-b22l);
else
    NSCEL=0;
end
%===============
 %MF3: ZCEU
if ce>=a23u & ce<=b23u
    ZCEU=(a23u-ce)/(a23u-b23u);
elseif ce>=b23u & ce<=c23u
    ZCEU=(c23u-ce)/(c23u-b23u);
else
    ZCEU=0;
end 
%MF3: ZCEL
if ce>=a23l & ce<=b23l
    ZCEL=(a23l-ce)/(a23l-b23l);
elseif ce>=b23l & ce<=c23l
    ZCEL=(c23l-ce)/(c23l-b23l);
else
    ZCEL=0;
end 
%===============
 %MF4: PSCEU
if ce>=a24u & ce<=b24u
    PSCEU=(a24u-ce)/(a24u-b24u);
elseif ce>=b24u & ce<=c24u
    PSCEU=(c24u-ce)/(c24u-b24u);
else
    PSCEU=0;
end 
%MF4: PSCEL
if ce>=a24l & ce<=b24l
    PSCEL=(a24l-ce)/(a24l-b24l);
elseif ce>=b24l & ce<=c24l
    PSCEL=(c24l-ce)/(c24l-b24l);
else
    PSCEL=0;
end 
%===============
 %MF5: PLCEU
if ce>=a25u & ce<=b25u
    PLCEU=(a25u-ce)/(a25u-b25u);
elseif ce>=b25u & ce<=c25u
    PLCEU=(c25u-ce)/(c25u-b25u);
else
    PLCEU=0;
end 
 %MF5: PLCEL
if ce>=a25l & ce<=b25l
    PLCEL=(a25l-ce)/(a25l-b25l);
elseif ce>=b25l & ce<=c25l
    PLCEL=(c25l-ce)/(c25l-b25l);
else
    PLCEL=0;
end 
%========================================    
% Rules
r1u=min(PLEU,NLCEU);            r1l=min(PLEL,NLCEL);
r2u=min(PLEU,NSCEU);            r2l=min(PLEL,NSCEL);
r3u=min(PLEU,ZCEU);             r3l=min(PLEL,ZCEL);
r4u=min(PLEU,PSCEU);            r4l=min(PLEL,PSCEL);
r5u=min(PLEU,PLCEU);            r5l=min(PLEL,PLCEL);
r6u=min(PSEU,NLCEU);            r6l=min(PSEL,NLCEL);
r7u=min(PSEU,NSCEU);            r7l=min(PSEL,NSCEL);
r8u=min(PSEU,ZCEU);             r8l=min(PSEL,ZCEL);
r9u=min(PSEU,PSCEU);            r9l=min(PSEL,PSCEL);
r10u=min(PSEU,PLCEU);           r10l=min(PSEL,PLCEL);
r11u=min(ZEU,NLCEU);            r11l=min(ZEL,NLCEL);
r12u=min(ZEU,NSCEU);            r12l=min(ZEL,NSCEL);
r13u=min(ZEU,ZCEU);             r13l=min(ZEL,ZCEL);
r14u=min(ZEU,PSCEU);            r14l=min(ZEL,PSCEL);
r15u=min(ZEU,PLCEU);            r15l=min(ZEL,PLCEL);
r16u=min(NSEU,NLCEU);           r16l=min(NSEL,NLCEL);
r17u=min(NSEU,NSCEU);           r17l=min(NSEL,NSCEL);
r18u=min(NSEU,ZCEU);            r18l=min(NSEL,ZCEL);
r19u=min(NSEU,PSCEU);           r19l=min(NSEL,PSCEL);
r20u=min(NSEU,PLCEU);           r20l=min(NSEL,PLCEL);
r21u=min(NLEU,NLCEU);           r21l=min(NLEL,NLCEL);
r22u=min(NLEU,NSCEU);           r22l=min(NLEL,NSCEL);
r23u=min(NLEU,ZCEU);            r23l=min(NLEL,ZCEL);
r24u=min(NLEU,PSCEU);           r24l=min(NLEL,PSCEL);
r25u=min(NLEU,PLCEU);           r25l=min(NLEL,PLCEL);

h1u=max(max(r3u,r4u),r5u);     h2u=max(max(r9u,r10u),r15u);
PLU=max(h1u,h2u);
h1l=max(max(r3l,r4l),r5l);     h2l=max(max(r9l,r10l),r15l);
PLL=max(h1l,h2l);
%=========
h3u=max(max(r2u,r8u),max(r14u,r20u));
PSU=h3u;
h3l=max(max(r2l,r8l),max(r14l,r20l));
PSL=h3l;
%========
h4u=max(max(r1u,r7u),r13u);    h5u=max(r19u,r25u);
ZU=max(h4u,h5u);
h4l=max(max(r1l,r7l),r13l);    h5l=max(r19l,r25l);
ZL=max(h4l,h5l);
%=========
h6u=max(max(r6u,r12u),max(r18u,r24u));
NSU=h6u;
h6l=max(max(r6l,r12l),max(r18l,r24l));
NSL=h6l;
%=========
h7u=max(max(r11u,r16u),r17u);   h8u=max(max(r21u,r22u),r23u);
NLU=max(h7u,h8u);
h7l=max(max(r11l,r16l),r17l);   h8l=max(max(r21l,r22l),r23l);
NLL=max(h7l,h8l);
%=======================================
%OUTPUT PROCESSING for PD
% 1) Compute Centroid of M consequent IT2 FSs by using KM algorithms
 % the centroid of MF1
 % AD=[-1.5 -0.66 -0.66 -0.25 -0.95 -0.66 -0.66 -0.55  1];
 AD=[-1.5 -0.66 -0.66 -0.25 -0.95 -0.66 -0.66 -0.55  1];
 [CCD1, CD1l, CD1r]=centroidIT2(AD);
 % the centroid of MF2
 % BD=[-0.95 -0.33 -0.33 0.01 -0.65 -0.33 -0.33 -0.25 1];
 BD=[-0.95 -0.33 -0.33 0.01 -0.65 -0.33 -0.33 -0.25 1];
 [CCD2, CD2l, CD2r]=centroidIT2(BD);
 % the centroid of MF3
 % CD=[-0.33 0 0 0.33 -0.33 0 0 0.33 1];
 CD=[-0.33 0 0 0.33 -0.33 0 0 0.33 1];
 [CCD3, CD3l, CD3r]=centroidIT2(CD);
 % the centroid of MF4
 DD=[-0.25 0.33 0.33 0.7 0.01 0.33 0.33 0.4 1];
 [CCD4, CD4l, CD4r]=centroidIT2(DD);
 % the centroid of MF5
 ED=[0.05 0.66 0.66 1 0.4 0.66 0.66 0.7 1];
 [CCD5, CD5l, CD5r]=centroidIT2(ED);

num=(NLL*(CCD1))+(NSL*(-0.2))+(ZL*CCD3)+(PSL*CCD4)+(PLL*CCD5);
den=NLL+NSL+ZL+PSL+PLL;
if den == 0
    yLV=0;
else
    yLV=num/den;  
end
 num=(NLU*(CCD1))+(NSU*(-0.2))+(ZU*CCD3)+(PSU*CCD4)+(PLU*CCD5);
den=NLU+NSU+ZU+PSU+PLU;
if den == 0
    yRV=0;
else
    yRV=num/den;  
end
 % 5) Compute A pproximate Defuzzified Output
 cuvn=(yLV+yRV)/2;
%=============
%output gain

u0_v=kuv*cuvn;
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
%MF1: NLEU
if e>=a11u & e<=b11u
    NLEU=(a11u-e)/(a11u-b11u);
elseif e>=b11u & e<=c11u
    NLEU=(c11u-e)/(c11u-b11u);
else
    NLEU=0;
end
%MF1: NLEL
if e>=a11l & e<=b11l
    NLEL=(a11l-e)/(a11l-b11l);
elseif e>=b11l & e<=c11l
    NLEL=(c11l-e)/(c11l-b11l);
else
    NLEL=0;
end
%================
%MF2: NSEU
if e>=a12u & e<=b12u
    NSEU=(a12u-e)/(a12u-b12u);
elseif e>=b12u & e<=c12u
    NSEU=(c12u-e)/(c12u-b12u);
else
    NSEU=0;
end
%MF2: NSEL
if e>=a12l & e<=b12l
    NSEL=(a12l-e)/(a12l-b12l);
elseif e>=b12l & e<=c12l
    NSEL=(c12l-e)/(c12l-b12l);
else
    NSEL=0;
end
%===============
 %MF3: ZEU
if e>=a13u & e<=b13u
    ZEU=(a13u-e)/(a13u-b13u);
elseif e>=b13u & e<=c13u
    ZEU=(c13u-e)/(c13u-b13u);
else
    ZEU=0;
end 
 %MF3: ZEL
if e>=a13l & e<=b13l
    ZEL=(a13l-e)/(a13l-b13l);
elseif e>=b13l & e<=c13l
    ZEL=(c13l-e)/(c13l-b13l);
else
    ZEL=0;
end 
%===============
 %MF4: PSEU
if e>=a14u & e<=b14u
    PSEU=(a14u-e)/(a14u-b14u);
elseif e>=b14u & e<=c14u
    PSEU=(c14u-e)/(c14u-b14u);
else
    PSEU=0;
end 
 %MF4: PSEL
if e>=a14u & e<=b14u
    PSEL=(a14u-e)/(a14u-b14u);
elseif e>=b14u & e<=c14u
    PSEL=(c14u-e)/(c14u-b14u);
else
    PSEL=0;
end 
%===============
 %MF5: PLEU
if e>=a15u & e<=b15u
    PLEU=(a15u-e)/(a15u-b15u);
elseif e>=b15u & e<=c15u 
    PLEU=(c15u-e)/(c15u-b15u);
else
    PLEU=0;
end 
%MF5: PLE
if e>=a15u & e<=b15u
    PLEL=(a15u-e)/(a15u-b15u);
elseif e>=b15u & e<=c15u
    PLEL=(c15u-e)/(c15u-b15u);
else
    PLEL=0;
end 
%========================================
 %fuzzification of CE
%MF1: NLCEU
if ce>=a21u & ce<=b21u
    NLCEU=(a21u-ce)/(a21u-b21u);
elseif ce>=b21u & ce<=c21u
    NLCEU=(c21u-ce)/(c21u-b21u);
else
    NLCEU=0;
end
%MF1: NLCEL
if ce>=a21l & ce<=b21l
    NLCEL=(a21l-ce)/(a21l-b21l);
elseif ce>=b21l & ce<=c21l
    NLCEL=(c21l-ce)/(c21l-b21l);
else
    NLCEL=0;
end
%================
%MF2: NSCEU
if ce>=a22u & ce<=b22u
    NSCEU=(a22u-ce)/(a22u-b22u);
elseif ce>=b22u & ce<=c22u
    NSCEU=(c22u-ce)/(c22u-b22u);
else
    NSCEU=0;
end
%MF2: NSCEL
if ce>=a22l & ce<=b22l
    NSCEL=(a22l-ce)/(a22l-b22l);
elseif ce>=b22l & ce<=c22l
    NSCEL=(c22l-ce)/(c22l-b22l);
else
    NSCEL=0;
end
%===============
 %MF3: ZCEU
if ce>=a23u & ce<=b23u
    ZCEU=(a23u-ce)/(a23u-b23u);
elseif ce>=b23u & ce<=c23u
    ZCEU=(c23u-ce)/(c23u-b23u);
else
    ZCEU=0;
end 
%MF3: ZCEL
if ce>=a23l & ce<=b23l
    ZCEL=(a23l-ce)/(a23l-b23l);
elseif ce>=b23l & ce<=c23l
    ZCEL=(c23l-ce)/(c23l-b23l);
else
    ZCEL=0;
end 
%===============
 %MF4: PSCEU
if ce>=a24u & ce<=b24u
    PSCEU=(a24u-ce)/(a24u-b24u);
elseif ce>=b24u & ce<=c24u
    PSCEU=(c24u-ce)/(c24u-b24u);
else
    PSCEU=0;
end 
%MF4: PSCEL
if ce>=a24l & ce<=b24l
    PSCEL=(a24l-ce)/(a24l-b24l);
elseif ce>=b24l & ce<=c24l
    PSCEL=(c24l-ce)/(c24l-b24l);
else
    PSCEL=0;
end 
%===============
 %MF5: PLCEU
if ce>=a25u & ce<=b25u
    PLCEU=(a25u-ce)/(a25u-b25u);
elseif ce>=b25u & ce<=c25u
    PLCEU=(c25u-ce)/(c25u-b25u);
else
    PLCEU=0;
end 
 %MF5: PLCEL
if ce>=a25l & ce<=b25l
    PLCEL=(a25l-ce)/(a25l-b25l);
elseif ce>=b25l & ce<=c25l
    PLCEL=(c25l-ce)/(c25l-b25l);
else
    PLCEL=0;
end 
%========================================    
% Rules
r1u=min(PLEU,NLCEU);            r1l=min(PLEL,NLCEL);
r2u=min(PLEU,NSCEU);            r2l=min(PLEL,NSCEL);
r3u=min(PLEU,ZCEU);             r3l=min(PLEL,ZCEL);
r4u=min(PLEU,PSCEU);            r4l=min(PLEL,PSCEL);
r5u=min(PLEU,PLCEU);            r5l=min(PLEL,PLCEL);
r6u=min(PSEU,NLCEU);            r6l=min(PSEL,NLCEL);
r7u=min(PSEU,NSCEU);            r7l=min(PSEL,NSCEL);
r8u=min(PSEU,ZCEU);             r8l=min(PSEL,ZCEL);
r9u=min(PSEU,PSCEU);            r9l=min(PSEL,PSCEL);
r10u=min(PSEU,PLCEU);           r10l=min(PSEL,PLCEL);
r11u=min(ZEU,NLCEU);            r11l=min(ZEL,NLCEL);
r12u=min(ZEU,NSCEU);            r12l=min(ZEL,NSCEL);
r13u=min(ZEU,ZCEU);             r13l=min(ZEL,ZCEL);
r14u=min(ZEU,PSCEU);            r14l=min(ZEL,PSCEL);
r15u=min(ZEU,PLCEU);            r15l=min(ZEL,PLCEL);
r16u=min(NSEU,NLCEU);           r16l=min(NSEL,NLCEL);
r17u=min(NSEU,NSCEU);           r17l=min(NSEL,NSCEL);
r18u=min(NSEU,ZCEU);            r18l=min(NSEL,ZCEL);
r19u=min(NSEU,PSCEU);           r19l=min(NSEL,PSCEL);
r20u=min(NSEU,PLCEU);           r20l=min(NSEL,PLCEL);
r21u=min(NLEU,NLCEU);           r21l=min(NLEL,NLCEL);
r22u=min(NLEU,NSCEU);           r22l=min(NLEL,NSCEL);
r23u=min(NLEU,ZCEU);            r23l=min(NLEL,ZCEL);
r24u=min(NLEU,PSCEU);           r24l=min(NLEL,PSCEL);
r25u=min(NLEU,PLCEU);           r25l=min(NLEL,PLCEL);

h1u=max(max(r3u,r4u),r5u);     h2u=max(max(r9u,r10u),r15u);
PLU=max(h1u,h2u);
h1l=max(max(r3l,r4l),r5l);     h2l=max(max(r9l,r10l),r15l);
PLL=max(h1l,h2l);
%=========
h3u=max(max(r2u,r8u),max(r14u,r20u));
PSU=h3u;
h3l=max(max(r2l,r8l),max(r14l,r20l));
PSL=h3l;
%========
h4u=max(max(r1u,r7u),r13u);    h5u=max(r19u,r25u);
ZU=max(h4u,h5u);
h4l=max(max(r1l,r7l),r13l);    h5l=max(r19l,r25l);
ZL=max(h4l,h5l);
%=========
h6u=max(max(r6u,r12u),max(r18u,r24u));
NSU=h6u;
h6l=max(max(r6l,r12l),max(r18l,r24l));
NSL=h6l;
%=========
h7u=max(max(r11u,r16u),r17u);   h8u=max(max(r21u,r22u),r23u);
NLU=max(h7u,h8u);
h7l=max(max(r11l,r16l),r17l);   h8l=max(max(r21l,r22l),r23l);
NLL=max(h7l,h8l);
%=======================================
%OUTPUT PROCESSING for PD
% 1) Compute Centroid of M consequent IT2 FSs by using KM algorithms
 % the centroid of MF1
 AD=[-0.99 -0.66 -0.66 -0.25 -0.7 -0.66 -0.66 -0.35  1];
 [CCDw1, CD1l, CD1r]=centroidIT2(AD);
 % the centroid of MF2
 BD=[-0.7 -0.33 -0.33 0.25 -0.35 -0.33 -0.33 -0.001 1];
 [CCDw2, CD2l, CD2r]=centroidIT2(BD);
 % the centroid of MF3
 CD=[-0.33 0 0 0.33 -0.33 0 0 0.33 1];
 [CCDw3, CD3l, CD3r]=centroidIT2(CD);
 % the centroid of MF4
 DD=[-0.001 0.33 0.33 0.85 0.25 0.33 0.33 0.6 1];
 [CCDw4, CD4l, CD4r]=centroidIT2(DD);
 % the centroid of MF5
 ED=[0.3 0.66 0.66 1.5 0.5 0.66 0.66 0.95 1];
 [CCDw5, CD5l, CD5r]=centroidIT2(ED);

num=(NLL*(CCDw1))+(NSL*(-0.5))+(ZL*CCDw3)+(PSL*CCDw4)+(PLL*CCDw5);
den=NLL+NSL+ZL+PSL+PLL;
if den == 0
    yLW=0;
else
    yLW=num/den;  
end
 num=(NLU*(CCDw1))+(NSU*(-0.5))+(ZU*CCDw3)+(PSU*CCDw4)+(PLU*CCDw5);
den=NLU+NSU+ZU+PSU+PLU;
if den == 0
    yRW=0;
else
    yRW=num/den;  
end
 
 % 5) Compute A pproximate Defuzzified Output
 cuwn=(yLW+yRW)/2;
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
         vc = vc + 0.9;   
         wc = wc + 0.9;
     end

     % if k>=80
     %     vc = vc + 2 * sin(0.3*pi*k*T) + 2 * cos(0.3*pi*k*T);   
     %     wc = wc + 2 * sin(0.3*pi*k*T) + 2 * sin(0.3*pi*k*T);
     % end
     % 
     if k>=200 && k<=250
          dv = 2.9 * sin(0.3*pi*k*T) + 2.9 * cos(0.3*pi*k*T);
          dw = 2.9 * sin(0.3*pi*k*T) + 2.9 * sin(0.3*pi*k*T);
     end
     % 
     %  if k>=200 & k<=250
     %      dv = 0.8;
     %      dw = 0.8;
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

    Mae2(k) = (1/k) * sum(abs(EX+ EY + ETH));
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
title('Type 2 ADRC');
legend('Reference Trajectory','ADRC-FPD');
axis equal;
grid on;
savefig(fullfile(outputFolder, 'Trajectory_t2.fig'));

% 2. Desired vs Actual Linear Velocity
figure;
plot((0:N-1)*T, vd, 'b--', 'LineWidth', 2); hold on;
plot((0:N-1)*T, vm, 'r-', 'LineWidth', 2);
ylabel('Linear Velocity (m/s)');
xlabel('Time (s)');
title('Desired vs Actual Linear Velocity');
legend('vd','vm');
grid on;
savefig(fullfile(outputFolder, 'LinearVelocity_t2.fig'));

% 3. Desired vs Actual Angular Velocity
figure;
plot((0:N-1)*T, wd, 'b--', 'LineWidth', 2); hold on;
plot((0:N-1)*T, wm, 'r-', 'LineWidth', 2);
ylabel('Angular Velocity (rad/s)');
xlabel('Time (s)');
title('Desired vs Actual Angular Velocity');
legend('wd','wm');
grid on;
savefig(fullfile(outputFolder, 'AngularVelocity_t2.fig'));

% 4. Desired vs Actual Theta (Orientation)
% figure;
% plot((0:N-1)*T, theta_r, 'b--', 'LineWidth', 2); hold on;
% plot((0:N-1)*T, theta_m, 'r-', 'LineWidth', 2);
% ylabel('Orientation \theta (rad)');
% xlabel('Time (s)');
% title('Desired vs Actual Orientation \theta');
% legend('\theta_{ref}','\theta_{real}');
% grid on;
% savefig(fullfile(outputFolder, 'ThetaOrientation_t2.fig'));

% 5. MAE Error Over Time
position_errors = sqrt((xr - xm).^2 + (yr - ym).^2);
% figure;
% plot((0:N-1)*T, abs(position_errors), 'k-', 'LineWidth', 2);
% xlabel('Time (s)');
% ylabel('Absolute Position Error (m)');
% title('Mean Absolute Error Over Time');
% grid on;
% savefig(fullfile(outputFolder, 'MAE_PositionError_t2.fig'));
% 
% 6. X-axis Position: Reference vs Actual
% figure;
% plot((0:N-1)*T, xr, 'b--', 'LineWidth', 2); hold on;
% plot((0:N-1)*T, xm, 'r-', 'LineWidth', 2);
% xlabel('Time (s)');
% ylabel('X Position (m)');
% title('X-axis: Reference vs Actual');
% legend('X_{ref}', 'X_{actual}');
% grid on;
% savefig(fullfile(outputFolder, 'X_Position_t2.fig'));
% 
% 7. Y-axis Position: Reference vs Actual
% figure;
% plot((0:N-1)*T, yr, 'b--', 'LineWidth', 2); hold on;
% plot((0:N-1)*T, ym, 'r-', 'LineWidth', 2);
% xlabel('Time (s)');
% ylabel('Y Position (m)');
% title('Y-axis: Reference vs Actual');
% legend('Y_{ref}', 'Y_{actual}');
% grid on;
% savefig(fullfile(outputFolder, 'Y_Position_t2.fig'));


% MAE (Mean Absolute Error)
MAE = mean(abs(position_errors));

% RMSE (Root Mean Square Error)
RMSE = sqrt(mean(position_errors.^2));

% Display results
fprintf('--- Error Metrics ---\n');
fprintf('RMSE = %.4f m\n', RMSE);
fprintf('MAE  = %.4f m\n', MAE);
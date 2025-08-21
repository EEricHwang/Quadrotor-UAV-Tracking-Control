
clear all
close all
clc

% [1] Author : Sounghwan Hwang 
% [2] Objective : This is a simulation code for T-S fuzzy model-based quadrotor tracking control
% [3] Date : August 20, 2025
% [4] Target Conference : Scitech 2026
% [5] Deadline : December 2, 2025

addpath("LMIs/")

cnt1                =        1;
cnt2                =        1;

mode                =        1;

dT_position         =        0.1;
dT_attitude         =        0.01;                                       % [-] Inner loop controller is 10 times faster than outer-loop controller.              
dT_altitude         =        0.1;

tspan               =        [0 30];

%% LMI Modules

skip_a_control      =        true;                                       % [-] Attitude controller

skip_z_control      =        true;                                       % [-] Altitude controller
skip_z_error        =        true;
skip_z_detector     =        true;
skip_z_reach        =        true;

skip_p_control      =        true;                                       % [-] Position controller
skip_p_error        =        true;
skip_p_detector     =        true;
skip_p_reach        =        true;

%% Quadrotor UAV dynamics

m                   =        1;                                          % [-] Quadrotor Mass 
g                   =        9.81;                                       % [-] Gravity
I_x                 =        0.0232;                                     % [-] Moment of Inertia (X axis)
I_y                 =        0.0232;                                     % [-] Moment of Inertia (Y axis)
I_z                 =        0.04;                                       % [-] Moment of Inertia (Z axis)
l                   =        1;                                          % [-] Quadrotor arm length

a1                  =        (I_y-I_z)/I_x;
a2                  =        (I_z-I_x)/I_y;
a3                  =        (I_x-I_y)/I_z;

b1                  =        l/I_z;
b2                  =        l/I_x;
b3                  =        l/I_y;

%% 

% ========================================================================
% ============= [1] Attitude Model Parameters (자세모델) ==================
% ========================================================================

% Roll, Roll_dot, Pitch, Pitch_dot, Yaw, Yaw_dot

ra                  =        4;                                          % [-]    Number of fuzzy rules 

rho1_max            =        2000*pi/180;                                % [-]    35 degree [rad/s]
rho1_min            =        -2000*pi/180;                               % [-]    35 degree [rad/s]
rho2_max            =        2000*pi/180;                                % [-]    35 degree [rad/s]
rho2_min            =        -2000*pi/180;                               % [-]    35 degree [rad/s]

Aa{1}               =        [1   dT_attitude  0             0              0                         0; 
                              0             1  0             0              0   a1*rho1_min*dT_attitude;
                              0             0  1   dT_attitude              0                         0;
                              0             0  0             1              0   a2*rho2_min*dT_attitude;
                              0             0  0             0              1               dT_attitude;
                              0             0  0   a3*rho2_min*dT_attitude  0                        1];

Aa{2}               =        [1   dT_attitude  0             0              0                         0; 
                              0             1  0             0              0   a1*rho1_min*dT_attitude;
                              0             0  1   dT_attitude              0                         0;
                              0             0  0             1              0   a2*rho2_max*dT_attitude;
                              0             0  0             0              1               dT_attitude;
                              0             0  0   a3*rho2_max*dT_attitude  0                        1];

Aa{3}               =        [1   dT_attitude  0             0              0                         0; 
                              0             1  0             0              0   a1*rho1_max*dT_attitude;
                              0             0  1   dT_attitude              0                         0;
                              0             0  0             1              0   a2*rho2_min*dT_attitude;
                              0             0  0             0              1               dT_attitude;
                              0             0  0   a3*rho2_min*dT_attitude  0                        1];

Aa{4}               =        [1   dT_attitude  0             0              0                         0; 
                              0             1  0             0              0   a1*rho1_max*dT_attitude;
                              0             0  1   dT_attitude              0                         0;
                              0             0  0             1              0   a2*rho2_max*dT_attitude;
                              0             0  0             0              1               dT_attitude;
                              0             0  0   a3*rho2_max*dT_attitude  0                        1];

Ba{1}               =        [0 0 0; dT_attitude*b1 0 0; 0 0 0; 0 dT_attitude*b2 0; 0 0 0; 0 0 dT_attitude*b3];
Ba{2}               =        Ba{1};
Ba{3}               =        Ba{1};
Ba{4}               =        Ba{1};

Ca                  =        [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0]; 
Bd_a                =        0.01*eye(6);
E_a                 =        0.01*eye(3);

UAV.Aa              =        Aa;
UAV.Ba              =        Ba;
UAV.Ca              =        Ca;
UAV.Bd_a            =        Bd_a;
UAV.E_a             =        E_a;

if ~skip_a_control
    [Ka, La]        =        LMI_Attitude_Control(UAV);                  % [-] Controller and state estimator gains for UAV position control
    save('Attitude_Control.mat','Ka','La');
else
    load('Attitude_Control.mat')
end

% 자세 모델 Reference Dynamics

Aa_ref              =        [1   dT_attitude             0             0     0               0;          % [-] Roll
                              0             1             0             0     0               0;          % [-] Roll_dot
                              0             0             1   dT_attitude     0               0;          % [-] Pitch
                              0             0             0             1     0               0;          % [-] Pitch_dot
                              0             0             0             0     1     dT_attitude;          % [-] Yaw
                              0             0             0             0     0              1];          % [-] Yaw_dot

Ba_ref              =        [0 0 0; dT_attitude 0 0; 0 0 0; 0 dT_attitude 0; 0 0 0; 0 0 dT_attitude];

Ka_p                =        -300;
Ka_d                =        -100;

Ka_gain             =       [Ka_p   Ka_d      0      0      0      0; 
                                0      0   Ka_p   Ka_d      0      0; 
                                0      0      0      0   Ka_p   Ka_d];

Aa_r                =        Aa_ref + Ba_ref*Ka_gain;

% Initial Values

x0_a                =        zeros(6,1);                                 % [-] Attitude real 
xhat0_a             =        zeros(6,1);                                 % [-] Attitude estimates
xr0_a               =        zeros(6,1);                                 % [-] Attitude reference

x_atti(:,cnt1)      =        x0_a;
xhat_atti(:,cnt1)   =        xhat0_a;
xr_atti(:,cnt1)     =        xr0_a;

%%

% =========================================================================
% ============= [2] Altitude Model Parameters (고도 모델) ==================
% =========================================================================

ral                 =        2;

Al_min              =        0.25/m;
Al_max              =        1/m;

Az{1}               =        [1 dT_altitude; 0 1];
Az{2}               =        Az{1};

Bz{1}               =        dT_altitude*[0;Al_min];
Bz{2}               =        dT_altitude*[0;Al_max];

Cz                  =        eye(2,2);
Bd_z                =        0.01*eye(2);                                % [-] Altitude Plant Process Noise Matrix
E_z                 =        0.01*eye(2);                                % [-] Altitude Sensor Measurement Noise Matrix
Gamma_z             =        [0.1;-0.1];                                 % [-] Altitude Stealthy Attack Matrix

Wbar_z              =        1;
Vbar_z              =        1;

UAV.Az              =        Az;
UAV.Bz              =        Bz;
UAV.Cz              =        Cz;
UAV.Bd_z            =        Bd_z;
UAV.E_z             =        E_z;
UAV.Gamma_z         =        Gamma_z;
UAV.Wbar_z          =        Wbar_z;
UAV.Vbar_z          =        Vbar_z;

%.. [2-1] Controller/Estimator

if ~skip_z_control
    [Kz, Lz, P2_z]  =        LMI_Altitude_Control(UAV);                  % [-] Controller and state estimator gains for UAV position control...
    save('Altitude_Control.mat','Kz','Lz','P2_z');
else
    load('Altitude_Control.mat')
end

UAV.Kz              =        Kz;
UAV.Lz              =        Lz;
UAV.P2_z            =        P2_z;

% 고도 모델 Reference Dynamics

Az_ref              =        [1 dT_altitude; 0 1];
Bz_ref              =        [0;dT_altitude];

Kz_p                =        -2;
Kz_d                =        -4;
Kz_gain             =        [Kz_p Kz_d];

Az_r                =        Az_ref + Bz_ref*Kz_gain;

% Initial Values

x0_z                =        zeros(2,1);                                    % [-] Attitude real 
xhat0_z             =        zeros(2,1);                                    % [-] Attitude estimates
xr0_z               =        zeros(2,1);                                    % [-] Attitude reference

x_alti(:,cnt2)      =        x0_z;
xhat_alti(:,cnt2)   =        xhat0_z;
xr_alti(:,cnt2)     =        xr0_z;

%%

% =========================================================================
% ============= [3] Position Model Parameters (위치 모델) =================
% =========================================================================

% X, Y, X_dot, Y_dot

rp                  =        2;

S1                  =        0.1/m;
S2                  =        50/m;

Ap{1}               =        [1 0 dT_position 0; 0 1 0 dT_position; 0 0 1 0; 0 0 0 1];
Ap{2}               =        [1 0 dT_position 0; 0 1 0 dT_position; 0 0 1 0; 0 0 0 1];

Bp{1}               =        [0 0; 0 0; dT_position*S1 0; 0 dT_position*S1];
Bp{2}               =        [0 0; 0 0; dT_position*S2 0; 0 dT_position*S2];

Cp                  =        [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];          % [-] x(k) y(k) Vx(k) Vy(k)  
Bd_p                =        0.01*eye(4);                                
E_p                 =        0.01*eye(4);                                      
Gamma_p             =        [0.25 -0.53 0.25 -0.77]';
Wbar_p              =        0.1;
Vbar_p              =        1;

UAV.Ap              =        Ap;
UAV.Bp              =        Bp;
UAV.Cp              =        Cp;
UAV.Bd_p            =        Bd_p;
UAV.E_p             =        E_p;
UAV.Gamma_p         =        Gamma_p;
UAV.Wbar_p          =        Wbar_p;
UAV.Vbar_p          =        Vbar_p;

%.. [3-1] Controller/Estimator

if ~skip_p_control
    [Kp, Lp, P2_p]  =        LMI_Position_Control(UAV);                     % [-] Controller and state estimator gains for UAV position control
    save('Position_Control.mat','Kp','Lp','P2_p');
else
    load('Position_Control.mat')
end

UAV.Kp              =        Kp;
UAV.Lp              =        Lp;
UAV.P2_p            =        P2_p;

% 위치 모델 Reference Dynamics

Ap_ref              =        [1  0  dT_position            0; 
                              0  1            0  dT_position;
                              0  0            1            0;
                              0  0            0           1];

Bp_ref              =        [0 0; 0 0; dT_position 0; 0 dT_position];

Kp_p                =        -2;
Kp_d                =        -3;
Kp_gain             =        [Kp_p 0 Kp_d 0; 0 Kp_p 0 Kp_d];

Ap_r                =        Ap_ref + Bp_ref*Kp_gain;

% Initial Values

x0_p                =        zeros(4,1);
xhat0_p             =        zeros(4,1);
xr0_p               =        zeros(4,1);

roll                =        xhat0_a(1);
pitch               =        xhat0_a(3);
yaw                 =        xhat0_a(5);
x_posi_init         =        xhat0_p(1);
y_posi_init         =        xhat0_p(2);
z_posi_init         =        xhat0_z(1);

%% 3D Simulation

Rotational          =        [ cos(yaw)*cos(pitch)   -sin(yaw)*cos(roll)+cos(yaw)*sin(pitch)*sin(roll)    sin(yaw)*sin(roll)+cos(yaw)*sin(pitch)*cos(roll);
                                sin(yaw)*cos(pitch)   cos(yaw)*cos(roll)+sin(yaw)*sin(pitch)*sin(roll)   -cos(yaw)*sin(roll)+sin(yaw)*sin(pitch)*cos(roll);
                                        -sin(pitch)                               cos(pitch)*sin(roll)                               cos(pitch)*cos(roll) ];

drone1(:,cnt2)      =        Rotational*[1 0 0]' + [x_posi_init y_posi_init z_posi_init]';
drone2(:,cnt2)      =        Rotational*[0 -1 0]' + [x_posi_init y_posi_init z_posi_init]';
drone3(:,cnt2)      =        Rotational*[-1 0 0]' + [x_posi_init y_posi_init z_posi_init]';
drone4(:,cnt2)      =        Rotational*[0 1 0]'+ [x_posi_init y_posi_init z_posi_init]';

x_posi(:,cnt2)      =        x0_p;
xhat_posi(:,cnt2)   =        xhat0_p;
xr_posi(:,cnt2)     =        xr0_p;  

roll_d(:,cnt2)      =        0;
pitch_d(:,cnt2)     =        0;
yaw_d(:,cnt2)       =        0;

T_atti(:,cnt1)      =        0;
T_posi(:,cnt2)      =        0;

%% Outer Loop Controller
for idx_outer = tspan(1) + dT_altitude : dT_altitude : tspan(2)
    
    %% [0] Reference signal

    if (idx_outer>=0) && (idx_outer<=10)
        Px         =  0;
        Py         =  0;
        Pz         =  20;
    elseif (idx_outer>10) && (idx_outer<=15)
        Px         =  10;
        Py         =  0;
        Pz         =  20;
    elseif (idx_outer>15) && (idx_outer<=20)
        Px         =  10;
        Py         =  10;
        Pz         =  20;    
    elseif (idx_outer>21) && (idx_outer<=25)
        Px         =  10;
        Py         =  10;
        Pz         =  30;    
    elseif (idx_outer>25) && (idx_outer<=30)
        Px         =  0;
        Py         =  10;
        Pz         =  30;   
    end

    ref_altitude   =  Pz;
    ref_position   =  [Px;Py];

    xr_z = Az_r*xr0_z - Kz_p*Bz_ref*ref_altitude;
    xr_p = Ap_r*xr0_p - Kp_p*Bp_ref*ref_position;       

    %% [1] UAV Altitude Controller

    Al_min     =  0.25/m;
    Al_max     =  1/m;

    AL_min     =  (Al_max-(cos(x0_a(1))*cos(x0_a(3)))/m)/(Al_max-Al_min); 
    AL_max     =  1-AL_min;

    N{1}       =  AL_min;
    N{2}       =  AL_max;

    sum_alti   =  N{1}+N{2};

    for i = 1 : ral
        w{i}   =  N{i}/sum_alti;
    end
    
    wal        =  -1+2*rand(2,1);
    val        =  -1+2*rand(2,1);

    x_z        =  zeros(2,1);
    xhat_z     =  zeros(2,1);

    y_z        =  Cz*x0_z + E_z*val; 
    yhat_z     =  Cz*xhat0_z;

    for i = 1 : ral
        for j = 1 : ral
            x_z = x_z + w{i}*w{j}*( Az{i}*x0_z + Bz{i}*Kz{j}*(xhat0_z - xr0_z) + Bd_z*wal );                       
            xhat_z = xhat_z + w{i}*w{j}*( Az{i}*xhat0_z + Bz{i}*Kz{j}*(xhat0_z - xr0_z) + Lz{i}*(y_z-yhat_z) );    
        end
    end

    %% [2] UAV Position Controller

    Po_min     =  0.1/m;   
    Po_max     =  20/m;
    uz         =  0;

    for i = 1 : ral
        uz = uz + w{i}*Kz{i}*( xhat0_z-xr0_z );
    end

    ut = (m*g)/(cos(xhat0_a(1))*cos(xhat0_a(3))) + uz;

    Position_min  =  (Po_max-ut/m)/(Po_max-Po_min);
    Position_max  =  1-Position_min;

    P{1}          =  Position_min;
    P{2}          =  Position_max;
    
    sum_posi      =  P{1}+P{2};
    up            =  zeros(2,1);

    for i = 1 : rp
        b{i} = P{i}/sum_posi;
        up = up + b{i}*Kp{i}*(xhat0_p-xr0_p);
    end

    wp            =  -0.1+0.2*rand(4,1);
    vp            =  -0.1+0.2*rand(4,1);

    x_p           =  zeros(4,1);
    xhat_p        =  zeros(4,1);

    y_p           =  Cp*x0_p + E_p*vp;    
    yhat_p        =  Cp*xhat0_p;

    for i = 1 : rp
        for j = 1 : rp
            x_p = x_p + b{i}*b{j}*( Ap{i}*x0_p + Bp{i}*Kp{j}*(xhat0_p - xr0_p) + Bd_p*wp ); 
            xhat_p = xhat_p + b{i}*b{j}*( Ap{i}*xhat0_p + Bp{i}*Kp{j}*( xhat0_p - xr0_p ) + Lp{i}*(y_p-yhat_p) ); 
        end
    end

    ux              =  up(1);
    uy              =  up(2);

    Roll_d          =  real(asin( sin(xhat0_a(5))*ux-cos(xhat0_a(5))*uy ));
    Pitch_d         =  real(asin( (cos(xhat0_a(5))*ux+sin(xhat0_a(5))*uy)/cos(Roll_d) ));
    Yaw_d           =  0;    

    ref_attitude    =  [Roll_d;Pitch_d;Yaw_d];

    %% Inner Loop Controller
    for idx_inner = idx_outer + dT_attitude : dT_attitude : idx_outer+dT_altitude

        xr_a      =  Aa_r*xr0_a - Ka_p*Ba_ref*ref_attitude;

        Rho1_min  =  (rho1_max-x0_a(4))/(rho1_max-rho1_min);
        Rho1_max  =  1-Rho1_min;

        Rho2_min  =  (rho2_max-x0_a(2))/(rho2_max-rho2_min);
        Rho2_max  =  1-Rho2_min;

        MU{1}     =  Rho1_min * Rho2_min;
        MU{2}     =  Rho1_max * Rho2_min;
        MU{3}     =  Rho1_min * Rho2_max;
        MU{4}     =  Rho1_max * Rho2_max;

        sum_atti  =  MU{1}+MU{2}+MU{3}+MU{4};

        for i = 1 : ra
            h{i}  =  real(MU{i}/sum_atti);
        end

        wa        =  -0.01+0.02*rand(6,1);
        va        =  -0.01+0.02*rand(3,1);

        x_a       =  zeros(6,1);
        xhat_a    =  zeros(6,1);

        y_a       =  Ca*x0_a + E_a*va;
        yhat_a    =  Ca*xhat0_a;

        for i = 1 : ra
            for j = 1 : ra
                x_a = x_a + h{i}*h{j}*( Aa{i}*x0_a + Ba{i}*Ka{j}*(xhat0_a - xr0_a) + Bd_a*wa );
                xhat_a = xhat_a + h{i}*h{j}*( Aa{i}*xhat0_a + Ba{i}*Ka{j}*(xhat0_a - xr0_a) + La{i}*(y_a-yhat_a) );
            end
        end

        % Update UAV Attitude
        
        x0_a                  =   x_a;
        xhat0_a               =   xhat_a; 
        xr0_a                 =   xr_a;

        % Save UAV Attitude
        
        member_h1(:,cnt1+1)   =  h{1};
        member_h2(:,cnt1+1)   =  h{2};
        member_h3(:,cnt1+1)   =  h{3};
        member_h4(:,cnt1+1)   =  h{4};

        x_atti(:,cnt1+1)      =   x_a;
        xhat_atti(:,cnt1+1)   =   xhat_a;
        xr_atti(:,cnt1+1)     =   xr_a;

        T_atti(:,cnt1+1)      =   idx_inner;
        cnt1                  =   cnt1 + 1;

    end

    %% Quadrotor 3D simulation

    roll    = xhat0_a(1);
    pitch   = xhat0_a(3);
    yaw     = xhat0_a(5);
    X_posi  = xhat0_p(1);
    Y_posi  = xhat0_p(2);
    Z_posi  = xhat0_z(1);

    Rotational = [cos(yaw)*cos(pitch)  -sin(yaw)*cos(roll)+cos(yaw)*sin(pitch)*sin(roll)    sin(yaw)*sin(roll)+cos(yaw)*sin(pitch)*cos(roll);
                  sin(yaw)*cos(pitch)   cos(yaw)*cos(roll)+sin(yaw)*sin(pitch)*sin(roll)   -cos(yaw)*sin(roll)+sin(yaw)*sin(pitch)*cos(roll);
                          -sin(pitch)                               cos(pitch)*sin(roll)                               cos(pitch)*cos(roll)];

    drone1(:,cnt2)      =   Rotational*[1 0 0]'  + [X_posi Y_posi Z_posi]';
    drone2(:,cnt2)      =   Rotational*[0 -1 0]' + [X_posi Y_posi Z_posi]';
    drone3(:,cnt2)      =   Rotational*[-1 0 0]' + [X_posi Y_posi Z_posi]';
    drone4(:,cnt2)      =   Rotational*[0 1 0]'  + [X_posi Y_posi Z_posi]';

    % Update UAV Position and Altitude
    
    xr0_z                 =   xr_z;
    xr0_p                 =   xr_p;

    x0_z                  =   x_z;
    xhat0_z               =   xhat_z;

    x0_p                  =   x_p;
    xhat0_p               =   xhat_p;

    % Save UAV Position and Altitude
    
    member_w1(:,cnt2+1)   =   w{1};
    member_w2(:,cnt2+1)   =   w{2};

    u_alti(:,cnt2+1)      =   ut;

    x_alti(:,cnt2+1)      =   x_z;                                       
    xhat_alti(:,cnt2+1)   =   xhat_z;    

    x_posi(:,cnt2+1)      =   x_p;       
    xhat_posi(:,cnt2+1)   =   xhat_p;    
    
    xr_alti(:,cnt2+1)     =   xr_z;                                      
    xr_posi(:,cnt2+1)     =   xr_p;       

    T_posi(:,cnt2+1)      =   idx_outer;
    cnt2                  =   cnt2 + 1;

end

Quadrotor{1}              =    drone1;
Quadrotor{2}              =    drone2;
Quadrotor{3}              =    drone3;
Quadrotor{4}              =    drone4;

% Attitude

UAV_Result.x_atti         =    x_atti;
UAV_Result.xhat_atti      =    xhat_atti;

% Altitude

UAV_Result.x_alti         =    x_alti;
UAV_Result.xhat_alti      =    xhat_alti;

% Position

UAV_Result.x_posi         =    x_posi;
UAV_Result.xhat_posi      =    xhat_posi;

% Reference Input

UAV_Result.xr_atti        =    xr_atti;
UAV_Result.xr_alti        =    xr_alti;
UAV_Result.xr_posi        =    xr_posi;
 
UAV_Result.Quadrotor      =    Quadrotor;

%% Plot or Playing Video

UAV_Tracking_Plot(T_atti,T_posi,UAV_Result,mode)


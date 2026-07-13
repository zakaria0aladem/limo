%% limo_ctrl_params.m
% Parameters for limo_mocap_control.slx -- run BEFORE building/simulating.
% Also where you design gains for your research controllers.

%% ---- ROS 2 network ----
P.domainID   = 10;
P.mocapTopic = '/vrpn_mocap/Limo/pose';
P.cmdTopic   = '/cmd_vel';
P.Ts         = 0.05;          % [s] control period (20 Hz)

%% ---- Goal (world / map frame) ----
P.goal = [0.5; 0.0; 0];       % [x; y; theta_rad]  -- start CLOSE and SAFE

%% ---- Actuator limits (LIMO Pro) ----
P.v_max = 0.15;               % [m/s] START SLOW. Hardware max ~1.0.
P.w_max = 0.80;               % [rad/s]
P.arrive_tol   = 0.05;        % [m]
P.arrive_tol_w = deg2rad(5);  % [rad]

%% ---- Controller selection (Variant Subsystem) ----
%   1 = P go-to-goal   2 = PID   3 = LQR
CTRL = 1;

V_P   = Simulink.Variant('CTRL == 1');
V_PID = Simulink.Variant('CTRL == 2');
V_LQR = Simulink.Variant('CTRL == 3');

%% ---- 1) P go-to-goal gains ----
P.Kp_rho   = 0.8;
P.Kp_alpha = 1.6;
P.Kp_theta = 1.2;

%% ---- 2) PID gains ----
P.Ki_rho   = 0.05;   P.Kd_rho   = 0.02;
P.Ki_alpha = 0.05;   P.Kd_alpha = 0.05;
P.i_limit  = 0.5;

%% ---- 3) LQR design ----
% Unicycle error dynamics in the ROBOT BODY frame, linearized about a
% nominal forward speed v0 (w0 = 0):
%
%   e = [ex; ey; etheta],   edot = A e + B u,   u = [v; w]
%
%   A = [0 0  0 ;      B = [-1  0 ;
%        0 0 v0;             0  0 ;
%        0 0  0 ]            0 -1]
%
% ey is controllable ONLY through the v0*etheta coupling -> v0 must be > 0.
% (At v0 = 0 the lateral direction is uncontrollable: a differential-drive
%  robot cannot slide sideways. That's the nonholonomic constraint.)
P.v0 = 0.25;

A = [0 0 0;
     0 0 P.v0;
     0 0 0];
B = [-1  0;
      0  0;
      0 -1];
Q = diag([4, 8, 2]);      % penalize ex, ey, etheta
R = diag([1, 0.5]);       % penalize v, w effort

% Control System Toolbox is installed -> use the built-in solver directly.
P.K_lqr = lqr(A, B, Q, R);      % 2x3 optimal gain

fprintf('LQR gain K =\n'); disp(P.K_lqr);
fprintf('  (baked into the LQR block automatically at build time)\n');

%% ---- Push to base workspace ----
assignin('base','P',P);
assignin('base','CTRL',CTRL);
assignin('base','V_P',V_P);
assignin('base','V_PID',V_PID);
assignin('base','V_LQR',V_LQR);

fprintf(['\nParams loaded. CTRL = %d  (1=P, 2=PID, 3=LQR)\n' ...
         'v_max = %.2f m/s -- raise only after a successful slow run.\n'], ...
         CTRL, P.v_max);

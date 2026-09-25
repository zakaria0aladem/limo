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

if exist('lqr','file') == 2 && license('test','Control_Toolbox')
    P.K_lqr = lqr(A, B, Q, R);          % 2x3 optimal gain, follows Q/R/v0
    fprintf('LQR gain K (from lqr) =\n');
else
    % Toolbox-free fallback: the lqr() result for the DEFAULT v0, Q, R above.
    % If you change v0/Q/R without the toolbox, this K will NOT follow.
    P.K_lqr = [-2.0000,       0,       0;
                     0, -4.0000, -2.4495];
    warning('limo_ctrl_params:noLqr', ...
        ['Control System Toolbox not available -- using the precomputed K ' ...
         'for v0=0.25, Q=diag([4 8 2]), R=diag([1 0.5]). Edits to Q/R/v0 are ignored.']);
    fprintf('LQR gain K (precomputed) =\n');
end
disp(P.K_lqr);
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

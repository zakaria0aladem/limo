function build_limo_control_model(mdl)
%BUILD_LIMO_CONTROL_MODEL  Build a Simulink model that closes the loop on
% OptiTrack mocap and drives the LIMO via /cmd_vel.
%
%   >> limo_ctrl_params          % load P, CTRL, variant objects FIRST
%   >> build_limo_control_model  % builds + opens limo_mocap_control.slx
%
% ARCHITECTURE (Nav2 is bypassed -- direct closed-loop control)
%
%   /vrpn_mocap/Limo/pose -> [Subscribe] -> [Bus Selector] -> quat2yaw
%                                                  |
%                                        state = [x; y; theta]
%                                                  |
%   goal = [xd; yd; thd] --------------> [ Variant Subsystem ]
%                                        |  P | PID | LQR    |   <-- swap here
%                                                  |
%                                             u = [v; w]
%                                                  |
%                                     [Saturate] -> [E-STOP]
%                                                  |
%                     [Blank Msg Twist] -> [Bus Assignment] -> [Publish /cmd_vel]
%
% SAFETY: two Manual Switch blocks act as an E-STOP and start in the ZERO
% (safe) position. Double-click them DURING simulation to enable motion.
%
% NOTE: add_block returns a numeric HANDLE, not a path. We therefore build
% path strings explicitly (a handle concatenated with '/state' produces
% garbage characters and add_block fails).

if nargin < 1, mdl = 'limo_mocap_control'; end

assert(evalin('base','exist(''P'',''var'')')==1, ...
    'Run limo_ctrl_params first (P not found in base workspace).');
P = evalin('base','P');

% ---------- fresh model ----------
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist([mdl '.slx'],'file'), delete([mdl '.slx']); end
new_system(mdl);
open_system(mdl);
set_param(mdl, ...
    'SolverType','Fixed-step', 'Solver','FixedStepDiscrete', ...
    'FixedStep', num2str(P.Ts), 'StopTime','inf');

%% ===== 1. ROS 2 Subscribe (mocap pose) =====
% Subscribe has TWO outputs:  port 1 = IsNew (bool),  port 2 = Msg (bus).
% Wiring port 1 into a Bus Selector gives "not a bus signal".
subP = [mdl '/Mocap Pose'];
add_block('ros2lib/Subscribe', subP, 'Position',[30 100 130 150]);
trySet(subP,'topicSource','Specify your own');
trySet(subP,'topic', P.mocapTopic);
trySet(subP,'messageType','geometry_msgs/PoseStamped');
trySet(subP,'sampleTime', num2str(P.Ts));
% QoS param names are UPPERCASE 'QOS...'  (lowercase silently does nothing).
% vrpn_mocap publishes BEST-EFFORT; a reliable subscriber will not match it.
trySet(subP,'QOSReliability','Best effort');
trySet(subP,'QOSDurability','Volatile');
trySet(subP,'QOSHistory','Keep last');
trySet(subP,'QOSDepth','5');

%% ===== 2. Unpack pose -> [x; y; theta] =====
% Bus signal selections are set LATER, after an Update Model creates the
% SL_Bus_* objects. Setting them now makes Simulink "remap" them to bogus
% names (e.g. linear.x -> x) against the block's default message type.
busP = [mdl '/Unpack Pose'];
add_block('simulink/Signal Routing/Bus Selector', busP, 'Position',[180 90 190 170]);

addMatlabFcn(mdl, 'quat2yaw', [250 150 330 200], quat2yawCode());

muxP = [mdl '/state'];
add_block('simulink/Signal Routing/Mux', muxP, 'Inputs','3','Position',[380 95 385 175]);

%% ===== 3. Goal =====
add_block('simulink/Sources/Constant', [mdl '/Goal'], ...
    'Value','P.goal', 'SampleTime', num2str(P.Ts), 'Position',[380 230 470 270]);

%% ===== 4. Modular controller (Variant Subsystem) =====
buildControllerVariant(mdl, [540 110 660 260]);

%% ===== 5. Saturation + E-STOP =====
add_block('simulink/Signal Routing/Demux', [mdl '/split u'], ...
    'Outputs','2','Position',[690 120 695 190]);
add_block('simulink/Discontinuities/Saturation', [mdl '/sat v'], ...
    'UpperLimit','P.v_max','LowerLimit','-P.v_max','Position',[720 110 750 140]);
add_block('simulink/Discontinuities/Saturation', [mdl '/sat w'], ...
    'UpperLimit','P.w_max','LowerLimit','-P.w_max','Position',[720 170 750 200]);
add_block('simulink/Sources/Constant', [mdl '/zero'], ...
    'Value','0','Position',[720 250 750 280]);
add_block('simulink/Signal Routing/Manual Switch', [mdl '/E-STOP v'], ...
    'Position',[800 110 830 160]);
add_block('simulink/Signal Routing/Manual Switch', [mdl '/E-STOP w'], ...
    'Position',[800 175 830 225]);

%% ===== 6. Build + publish Twist =====
blankP = [mdl '/Blank Twist'];
add_block('ros2lib/Blank Message', blankP, 'Position',[800 320 900 360]);
trySet(blankP,'messageType','geometry_msgs/Twist');
trySet(blankP,'sampleTime', num2str(P.Ts));

% AssignedSignals set after Update Model (see section 10).
basgP = [mdl '/Fill Twist'];
add_block('simulink/Signal Routing/Bus Assignment', basgP, 'Position',[950 130 960 360]);

pubP = [mdl '/cmd_vel'];
add_block('ros2lib/Publish', pubP, 'Position',[1020 220 1120 270]);
trySet(pubP,'topicSource','Specify your own');
trySet(pubP,'topic', P.cmdTopic);
trySet(pubP,'messageType','geometry_msgs/Twist');

%% ===== 7. Live display =====
xyP = [mdl '/Path (XY)'];
add_block('simulink/Sinks/XY Graph', xyP, 'Position',[460 330 520 390]);
trySet(xyP,'xmin','-1'); trySet(xyP,'xmax','4');
trySet(xyP,'ymin','-1'); trySet(xyP,'ymax','4');

add_block('simulink/Sinks/Scope', [mdl '/v, w'], ...
    'NumInputPorts','2','Position',[880 430 920 470]);
add_block('simulink/Sinks/Display', [mdl '/x'],    'Position',[420 420 470 440]);
add_block('simulink/Sinks/Display', [mdl '/y'],    'Position',[420 450 470 470]);
add_block('simulink/Sinks/Display', [mdl '/theta'],'Position',[420 480 470 500]);

try
    add_block('roslib/Simulation Rate Control', [mdl '/Rate Control'], ...
              'Position',[30 330 130 370]);
catch
    warning(['Simulation Rate Control block not added. Enable Simulation ' ...
             'Pacing (Run > Simulation Pacing) so the model runs at wall clock.']);
end

%% ===== 8. Wire =====
c = @(a,b) add_line(mdl, a, b, 'autorouting','on');
c('Mocap Pose/2','Unpack Pose/1');    % port 2 = Msg (bus). Port 1 = IsNew (bool)!
c('Unpack Pose/1','state/1');
c('Unpack Pose/2','state/2');
for k = 1:4
    c(sprintf('Unpack Pose/%d',k+2), sprintf('quat2yaw/%d',k));
end
c('quat2yaw/1','state/3');

c('state/1','Controller/1');
c('Goal/1','Controller/2');

c('Controller/1','split u/1');
c('split u/1','sat v/1');
c('split u/2','sat w/1');
c('sat v/1','E-STOP v/1');   c('zero/1','E-STOP v/2');
c('sat w/1','E-STOP w/1');   c('zero/1','E-STOP w/2');

c('Blank Twist/1','Fill Twist/1');
c('E-STOP v/1','Fill Twist/2');
c('E-STOP w/1','Fill Twist/3');
c('Fill Twist/1','cmd_vel/1');

c('Unpack Pose/1','Path (XY)/1');
c('Unpack Pose/2','Path (XY)/2');
c('Unpack Pose/1','x/1');
c('Unpack Pose/2','y/1');
c('quat2yaw/1','theta/1');
c('E-STOP v/1','v, w/1');
c('E-STOP w/1','v, w/2');

%% ===== 9. Annotate =====
try
    a = Simulink.Annotation([mdl '/note']);
    a.Text = sprintf(['LIMO closed-loop mocap control\n' ...
        'Run limo_ctrl_params first.\n' ...
        'Swap controller: CTRL = 1 (P) | 2 (PID) | 3 (LQR)\n' ...
        'E-STOP: double-click the Manual Switches during sim.\n' ...
        'Nav2 is NOT used -- this drives /cmd_vel directly.']);
    a.Position = [40 20 420 100];
catch
end

%% ===== 10. Create buses, THEN select bus signals =====
% ROS 2 message buses (SL_Bus_geometry_msgs_*) only exist after an Update
% Model. Assigning signals before that makes Simulink remap them to bogus
% top-level names, which then fail to resolve. Order matters.
set_param(mdl,'ZoomFactor','FitSystem');
try
    set_param(mdl,'SimulationCommand','update');   % creates the SL_Bus_* objects
catch ME
    warning('Update Model failed (%s). Press Ctrl+D manually, then re-run section 10.', ME.message);
end

try
    set_param([mdl '/Unpack Pose'],'OutputSignals', ...
        ['pose.position.x,pose.position.y,' ...
         'pose.orientation.x,pose.orientation.y,' ...
         'pose.orientation.z,pose.orientation.w']);
    set_param([mdl '/Fill Twist'],'AssignedSignals','linear.x,angular.z');
    set_param(mdl,'SimulationCommand','update');   % re-resolve with signals set
catch ME
    warning(['Could not set bus signals (%s).\n' ...
             'Fix by hand: double-click "Unpack Pose" -> select pose.position.x/y ' ...
             'and pose.orientation.x/y/z/w;  double-click "Fill Twist" -> select ' ...
             'linear.x and angular.z. Then Ctrl+D.'], ME.message);
end

save_system(mdl);

fprintf('\nBuilt %s.slx\n\n', mdl);
fprintf('Next:\n');
fprintf('  1) Simulation > ROS Toolbox > ROS Network -> Domain %d, rmw_fastrtps_cpp\n', P.domainID);
fprintf('  2) STOP the Nav2 stack (it fights over /cmd_vel).\n');
fprintf('  3) Press Run. E-STOP switches start SAFE -- double-click to enable motion.\n');
end

% ======================================================================
function buildControllerVariant(mdl, pos)
% Variant Subsystem, 3 interchangeable controllers, identical interface:
%   in1 = state [x;y;theta]   in2 = goal [xd;yd;thd]   out1 = u = [v;w]
vss = [mdl '/Controller'];
add_block('simulink/Ports & Subsystems/Variant Subsystem', vss, 'Position', pos);

% strip whatever the template put inside
inner = find_system(vss,'SearchDepth',1,'LookUnderMasks','all', ...
                    'MatchFilter',@Simulink.match.allVariants);
for i = 1:numel(inner)
    p = getfullname(inner{i});
    if ~strcmp(p, vss), try delete_block(p); catch, end, end
end

add_block('simulink/Sources/In1', [vss '/state'], 'Port','1','Position',[30 40 60 60]);
add_block('simulink/Sources/In1', [vss '/goal'],  'Port','2','Position',[30 120 60 140]);
add_block('simulink/Sinks/Out1',  [vss '/u'],     'Port','1','Position',[500 80 530 100]);

names = {'P Controller','PID Controller','LQR Controller'};
codes = {pCode(), pidCode(), lqrCode(evalin('base','P.K_lqr'))};
vars  = {'V_P','V_PID','V_LQR'};
y0 = 30;
for i = 1:3
    ss = [vss '/' names{i}];
    add_block('simulink/Ports & Subsystems/Subsystem', ss, ...
              'Position',[200 y0 340 y0+60]);
    kids = find_system(ss,'SearchDepth',1,'LookUnderMasks','all');
    for k = 1:numel(kids)
        p = getfullname(kids{k});
        if ~strcmp(p, ss), try delete_block(p); catch, end, end
    end

    add_block('simulink/Sources/In1', [ss '/state'],'Port','1','Position',[20 30 50 50]);
    add_block('simulink/Sources/In1', [ss '/goal'], 'Port','2','Position',[20 90 50 110]);
    add_block('simulink/Sinks/Out1',  [ss '/u'],    'Port','1','Position',[260 60 290 80]);
    addMatlabFcn(ss, 'ctrl', [120 45 200 95], codes{i});

    % wire INSIDE each choice subsystem (this is allowed)
    add_line(ss,'state/1','ctrl/1','autorouting','on');
    add_line(ss,'goal/1','ctrl/2','autorouting','on');
    add_line(ss,'ctrl/1','u/1','autorouting','on');

    % NOTE: do NOT add_line inside the Variant Subsystem itself. Simulink
    % connects each choice to the VSS Inports/Outports IMPLICITLY, by
    % matching port numbers. Drawing lines there raises:
    %   "Lines cannot be added to the Variant Subsystem block"

    trySet(ss,'VariantControl', vars{i});
    y0 = y0 + 90;
end
trySet(vss,'VariantControlMode','expression');
end

% ======================================================================
function addMatlabFcn(parent, name, pos, code)
% NOTE: sfroot is a FUNCTION -- `sfroot.find(...)` is invalid syntax.
% Assign it first, then call find(). If the code isn't set, the block keeps
% its default 1-input signature and later wiring to port 2 fails.
blkPath = [parent '/' name];
add_block('simulink/User-Defined Functions/MATLAB Function', blkPath, 'Position', pos);

sf  = sfroot;                                     % <-- was sfroot.find(...)
obj = find(sf, '-isa','Stateflow.EMChart', 'Path', blkPath);
if isempty(obj)
    error('addMatlabFcn:notFound', ...
          'Could not locate the MATLAB Function chart at %s', blkPath);
end
obj(1).Script = code;                             % sets code AND updates ports

% let Simulink refresh the port count before anything tries to wire it
try, set_param(bdroot(blkPath),'SimulationCommand','update'); catch, end
end

function trySet(blk, param, val)
% Sets a block mask parameter AND VERIFIES it actually took. Silently
% swallowing a failed set_param is exactly how "Blank Twist" reverted to its
% default geometry_msgs/Point message type in a past session -- the build
% "succeeded" with no error, and the bug only surfaced three steps later as a
% cryptic "not a bus signal" warning. Fail loudly here instead, at the source.
try
    dp = get_param(blk,'DialogParameters');
    if ~isempty(dp) && isfield(dp, param)
        set_param(blk, param, val);
    else
        set_param(blk, param, val);   % some params aren't in DialogParameters
    end
catch ME
    error('build_limo_control_model:paramSet', ...
        ['Failed to set "%s" = "%s" on block "%s".\n' ...
         'Reason: %s\n' ...
         'This parameter name may differ in your MATLAB release -- set it ' ...
         'by hand in the block dialog, then re-run this script.'], ...
        param, string(val), blk, ME.message);
end

% Verify: read the value back and compare. Message-type-like params are the
% ones that silently reverted before, so check any param whose current value
% doesn't match what we asked for.
try
    actual = get_param(blk, param);
    if ~isequal(actual, val) && ~isequal(string(actual), string(val))
        error('build_limo_control_model:paramMismatch', ...
            ['Set "%s" on block "%s" but it reads back as "%s" instead of "%s".\n' ...
             'This is the exact failure mode that caused Blank Twist to stay ' ...
             'geometry_msgs/Point in a past session. Set it by hand in the ' ...
             'block dialog (double-click the block), then re-run this script.'], ...
            param, blk, string(actual), string(val));
    end
catch ME2
    if strcmp(ME2.identifier, 'build_limo_control_model:paramMismatch')
        rethrow(ME2);
    end
    % param not readable via get_param (e.g. write-only) -- can't verify, skip
end
end

% ===================== MATLAB Function code =====================
function s = quat2yawCode()
s = sprintf([ ...
'function theta = quat2yaw(qx, qy, qz, qw)\n' ...
'%%#codegen\n' ...
'theta = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz));\n']);
end

function s = pCode()
s = sprintf([ ...
'function u = ctrl(state, goal)\n' ...
'%%#codegen\n' ...
'%% P go-to-goal for a unicycle (differential drive).\n' ...
'%% Edit gains here.\n' ...
'Kp_rho = 0.8; Kp_alpha = 1.6; Kp_theta = 1.2; tol = 0.05;\n' ...
'ex = goal(1) - state(1);\n' ...
'ey = goal(2) - state(2);\n' ...
'rho = sqrt(ex*ex + ey*ey);\n' ...
'if rho > tol\n' ...
'    alpha = wrapAng(atan2(ey, ex) - state(3));\n' ...
'    %% cos(alpha) prevents driving forward while badly misaligned\n' ...
'    v = Kp_rho * rho * cos(alpha);\n' ...
'    w = Kp_alpha * alpha;\n' ...
'else\n' ...
'    v = 0;\n' ...
'    w = Kp_theta * wrapAng(goal(3) - state(3));\n' ...
'end\n' ...
'u = [v; w];\n' ...
'\n' ...
'function a = wrapAng(a)\n' ...
'a = atan2(sin(a), cos(a));\n']);
end

function s = pidCode()
s = sprintf([ ...
'function u = ctrl(state, goal)\n' ...
'%%#codegen\n' ...
'%% PID go-to-goal. Integral state via persistent vars.\n' ...
'persistent iRho iAlpha ePrevRho ePrevAlpha\n' ...
'if isempty(iRho), iRho=0; iAlpha=0; ePrevRho=0; ePrevAlpha=0; end\n' ...
'Ts = 0.05;\n' ...
'Kp_rho=0.8; Ki_rho=0.05; Kd_rho=0.02;\n' ...
'Kp_a=1.6;  Ki_a=0.05;  Kd_a=0.05;\n' ...
'iLim = 0.5; tol = 0.05;\n' ...
'ex = goal(1) - state(1);\n' ...
'ey = goal(2) - state(2);\n' ...
'rho = sqrt(ex*ex + ey*ey);\n' ...
'if rho > tol\n' ...
'    alpha = wrapAng(atan2(ey, ex) - state(3));\n' ...
'    iRho   = max(-iLim, min(iLim, iRho   + rho*Ts));\n' ...
'    iAlpha = max(-iLim, min(iLim, iAlpha + alpha*Ts));\n' ...
'    dRho   = (rho   - ePrevRho)/Ts;\n' ...
'    dAlpha = (alpha - ePrevAlpha)/Ts;\n' ...
'    v = (Kp_rho*rho + Ki_rho*iRho + Kd_rho*dRho) * cos(alpha);\n' ...
'    w =  Kp_a*alpha + Ki_a*iAlpha + Kd_a*dAlpha;\n' ...
'    ePrevRho = rho; ePrevAlpha = alpha;\n' ...
'else\n' ...
'    iRho=0; iAlpha=0; ePrevRho=0; ePrevAlpha=0;\n' ...
'    v = 0;\n' ...
'    w = 1.2 * wrapAng(goal(3) - state(3));\n' ...
'end\n' ...
'u = [v; w];\n' ...
'\n' ...
'function a = wrapAng(a)\n' ...
'a = atan2(sin(a), cos(a));\n']);
end

function s = lqrCode(K)
% K (2x3) is baked in as a literal -- codegen cannot read P from the workspace.
if nargin < 1 || isempty(K), K = [-1.8 0 0; 0 -2.0 -2.2]; end
Kstr = sprintf('[%.6g %.6g %.6g; %.6g %.6g %.6g]', K(1,1),K(1,2),K(1,3), ...
                                                   K(2,1),K(2,2),K(2,3));
s = sprintf([ ...
'function u = ctrl(state, goal)\n' ...
'%%#codegen\n' ...
'%% LQR on unicycle error dynamics in the ROBOT BODY frame.\n' ...
'%% K was designed in limo_ctrl_params.m and baked in below.\n' ...
'%% Interface is fixed -- swap this body for MPC / DeePC / Koopman:\n' ...
'%%   in: state=[x;y;theta], goal=[xd;yd;thd]   out: u=[v;w]\n' ...
'K = %s;\n' ...
'v0 = 0.25; tol = 0.05;\n' ...
'th = state(3);\n' ...
'dx = goal(1) - state(1);\n' ...
'dy = goal(2) - state(2);\n' ...
'%% rotate world error into the body frame\n' ...
'ex =  cos(th)*dx + sin(th)*dy;\n' ...
'ey = -sin(th)*dx + cos(th)*dy;\n' ...
'eth = wrapAng(goal(3) - th);\n' ...
'rho = sqrt(dx*dx + dy*dy);\n' ...
'if rho > tol\n' ...
'    uu = -K * [ex; ey; eth];\n' ...
'    v = v0 + uu(1);\n' ...
'    w = uu(2);\n' ...
'else\n' ...
'    v = 0;\n' ...
'    w = 1.2 * eth;\n' ...
'end\n' ...
'u = [v; w];\n' ...
'\n' ...
'function a = wrapAng(a)\n' ...
'a = atan2(sin(a), cos(a));\n'], Kstr);
end

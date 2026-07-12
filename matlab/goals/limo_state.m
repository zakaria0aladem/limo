function s = limo_state(h, opts)
%LIMO_STATE  Read and print the LIMO's full state in one call.
%
%   s = limo_state(h);              % h from limo_connect()
%   s = limo_state(h, print=false); % silent, just return the struct
%
% Reports, side by side:
%   TRUTH     OptiTrack pose            (h.mocapSub)   -> [x y yaw]
%   ESTIMATE  Nav2 belief  map->base    (h.tftree)     -> [x y yaw]
%   ODOM      raw wheel odometry        (h.odomSub)    -> [x y yaw]
%   VELOCITY  body twist from /odom                    -> [v w]
%   BATTERY   pack voltage              (h.statusSub)  -> volts
%   DRIFT     ||TRUTH - ESTIMATE||                     -> meters
%
% Returns struct s with fields: truth, estimate, odom, vel, battery, drift.
%
% ---------------------------------------------------------------------------
% NOTE: This file was reconstructed from limo_connect.m's handle contract and
% the LimoStatus message (battery_voltage). It was referenced by the control-
% research notes but not in the original upload. Reconcile with your copy if
% one exists; the field names below match limo_connect()'s returned handle.
% ---------------------------------------------------------------------------

arguments
    h struct
    opts.print (1,1) logical = true
    opts.timeout (1,1) double = 2
end

s = struct('truth',[],'estimate',[],'odom',[],'vel',[],'battery',NaN,'drift',NaN);

% ----- TRUTH: OptiTrack pose -----
try
    p = receive(h.mocapSub, opts.timeout);
    s.truth = [p.pose.position.x, p.pose.position.y, ...
               yawFromQuat(p.pose.orientation)];
catch
    if opts.print, warning('No mocap pose (check vrpn_mocap / QoS / DDS).'); end
end

% ----- ESTIMATE: map -> base_link from the TF tree (Nav2 belief) -----
try
    tf = getTransform(h.tftree, h.mapFrame, h.baseFrame, ...
                      rostime('now',h.node), 'Timeout', opts.timeout);
    s.estimate = [tf.transform.translation.x, tf.transform.translation.y, ...
                  yawFromQuat(tf.transform.rotation)];
catch
    if opts.print, warning('No %s->%s transform yet.', h.mapFrame, h.baseFrame); end
end

% ----- ODOM: raw wheel odometry + body velocity -----
try
    o = receive(h.odomSub, opts.timeout);
    s.odom = [o.pose.pose.position.x, o.pose.pose.position.y, ...
              yawFromQuat(o.pose.pose.orientation)];
    s.vel  = [o.twist.twist.linear.x, o.twist.twist.angular.z];
catch
    if opts.print, warning('No /odom message.'); end
end

% ----- BATTERY: from /limo_status (optional; limo_msgs may be absent) -----
if ~isempty(h.statusSub)
    try
        st = receive(h.statusSub, opts.timeout);
        s.battery = st.battery_voltage;
    catch
    end
end

% ----- DRIFT: truth vs estimate -----
if numel(s.truth) == 3 && numel(s.estimate) == 3
    s.drift = hypot(s.truth(1)-s.estimate(1), s.truth(2)-s.estimate(2));
end

% ----- Print -----
if opts.print
    fprintf('\n--- LIMO state ---------------------------------------\n');
    printRow('TRUTH  (mocap)   ', s.truth);
    printRow('ESTIMATE (map->base)', s.estimate);
    printRow('ODOM   (wheels)  ', s.odom);
    if numel(s.vel) == 2
        fprintf('%-20s v = %+6.3f m/s   w = %+6.3f rad/s\n', 'VELOCITY', s.vel(1), s.vel(2));
    end
    if ~isnan(s.battery), fprintf('%-20s %.2f V\n', 'BATTERY', s.battery); end
    if ~isnan(s.drift),   fprintf('%-20s %.3f m (truth vs estimate)\n', 'DRIFT', s.drift); end
    fprintf('------------------------------------------------------\n');
end
end

% ===================== helpers =====================
function yaw = yawFromQuat(q)
yaw = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y^2 + q.z^2));
end

function printRow(label, xyt)
if numel(xyt) == 3
    fprintf('%-20s x = %+6.3f  y = %+6.3f  th = %+6.1f deg\n', ...
            label, xyt(1), xyt(2), rad2deg(xyt(3)));
else
    fprintf('%-20s (unavailable)\n', label);
end
end

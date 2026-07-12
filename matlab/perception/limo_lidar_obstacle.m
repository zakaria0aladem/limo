function obs = limo_lidar_obstacle(h, opts)
%LIMO_LIDAR_OBSTACLE  Read /scan and report the nearest obstacle + sector status.
%
%   obs = limo_lidar_obstacle(h);
%   obs = limo_lidar_obstacle(h, threshold=0.5, print=true);
%
% Pure MATLAB -- subscribes to /scan directly, no ROS node to deploy.
% (Dr. Bara asked for the LiDAR logic in MATLAB.)
%
% h comes from limo_connect(). A /scan subscriber is created once and cached.
%
% ANGLE CONVENTION (REP-103): 0 deg = straight ahead, + = LEFT, - = RIGHT.
% Sectors: FRONT = |angle| <= 30 deg, LEFT = +30..+90, RIGHT = -30..-90.
%
% Returns obs:
%   obs.distance    [m]    nearest valid range (Inf if none)
%   obs.angle_deg   [deg]  angle of the nearest obstacle
%   obs.side        'front'|'left'|'right'|'behind'|'none'
%   obs.is_obstacle logical  nearest < threshold
%   obs.front  .clear .min_dist    per-sector clearance
%   obs.left   .clear .min_dist
%   obs.right  .clear .min_dist
%   obs.scan   .ranges .angles     raw arrays (for plotting)
%   obs.ok     logical

arguments
    h struct
    opts.threshold (1,1) double = 0.5     % [m] "obstacle" if closer than this
    opts.scanTopic (1,1) string = "/scan"
    opts.print (1,1) logical = true
end

% cache the subscriber on first use
persistent scanSub
if isempty(scanSub)
    scanSub = ros2subscriber(h.node, opts.scanTopic, "sensor_msgs/LaserScan", ...
        "Reliability","besteffort","Durability","volatile","Depth",5);
    pause(0.5);
end

obs = struct('distance',Inf,'angle_deg',NaN,'side','none','is_obstacle',false, ...
    'front',struct('clear',true,'min_dist',Inf), ...
    'left', struct('clear',true,'min_dist',Inf), ...
    'right',struct('clear',true,'min_dist',Inf), ...
    'scan', struct('ranges',[],'angles',[]), 'ok',false);

try
    m = receive(scanSub, 3);
catch
    if opts.print, warning('No /scan message. Is the LiDAR running?'); end
    return
end

% reconstruct the angle of each range: angle(i) = angle_min + (i-1)*angle_increment
n = numel(m.ranges);
angles = m.angle_min + (0:n-1)' * m.angle_increment;   % [rad]
ranges = double(m.ranges(:));

% valid readings only (drop NaN, Inf, out-of-band)
valid = isfinite(ranges) & ranges >= m.range_min & ranges <= m.range_max;
r = ranges(valid);
a = angles(valid);
obs.scan.ranges = r;
obs.scan.angles = a;
obs.ok = true;

if isempty(r)
    if opts.print, fprintf('LiDAR: no valid returns\n'); end
    return
end

% nearest obstacle overall
[obs.distance, idx] = min(r);
obs.angle_deg = rad2deg(a(idx));
obs.is_obstacle = obs.distance < opts.threshold;
obs.side = sideOf(obs.angle_deg);

% per-sector minimum distance (front / left / right)
aDeg = rad2deg(a);
obs.front.min_dist = sectorMin(r, aDeg, -30,  30);
obs.left.min_dist  = sectorMin(r, aDeg,  30,  90);
obs.right.min_dist = sectorMin(r, aDeg, -90, -30);
obs.front.clear = obs.front.min_dist >= opts.threshold;
obs.left.clear  = obs.left.min_dist  >= opts.threshold;
obs.right.clear = obs.right.min_dist >= opts.threshold;

if opts.print
    fprintf('\nLiDAR nearest: %.2f m at %+.0f deg (%s)%s\n', ...
        obs.distance, obs.angle_deg, obs.side, ...
        ternary(obs.is_obstacle,'   <-- OBSTACLE',''));
    fprintf('  front %s (%.2f m) | left %s (%.2f m) | right %s (%.2f m)\n', ...
        clearStr(obs.front), obs.front.min_dist, ...
        clearStr(obs.left),  obs.left.min_dist, ...
        clearStr(obs.right), obs.right.min_dist);
end
end

% ---------- helpers ----------
function s = sideOf(deg)
if abs(deg) <= 30,      s = 'front';
elseif deg > 30 && deg <= 90,   s = 'left';
elseif deg < -30 && deg >= -90, s = 'right';
else,                   s = 'behind';
end
end

function d = sectorMin(r, aDeg, lo, hi)
in = aDeg >= lo & aDeg <= hi;
if any(in), d = min(r(in)); else, d = Inf; end
end

function s = clearStr(sec)
if sec.clear, s = 'CLEAR'; else, s = 'BLOCKED'; end
end

function out = ternary(c, a, b)
if c, out = a; else, out = b; end
end

function limo_goal(h, x, y, yawDeg, opts)
%LIMO_GOAL  Send a single navigation goal to Nav2 via /goal_pose.
%
%   limo_goal(h, 1.0, 0.5, 90)            % go to (1,0.5), face +y
%   limo_goal(h, 1.0, 0.5, 90, wait=true) % block until robot arrives
%
% Uses geometry_msgs/PoseStamped, which MATLAB ships built-in -- no
% ros2genmsg / nav2_msgs needed. Nav2 subscribes to /goal_pose and plans.
%
% h comes from limo_connect().

arguments
    h struct
    x (1,1) double
    y (1,1) double
    yawDeg (1,1) double = 0
    opts.wait (1,1) logical = false
    opts.tol (1,1) double = 0.20      % [m] arrival tolerance
    opts.timeout (1,1) double = 90    % [s] give up after this
    opts.frame (1,1) string = "map"
end

msg = ros2message(h.goalPub);
msg.header.frame_id = char(opts.frame);
msg.pose.position.x = x;
msg.pose.position.y = y;
msg.pose.position.z = 0;
yaw = deg2rad(yawDeg);
msg.pose.orientation.z = sin(yaw/2);
msg.pose.orientation.w = cos(yaw/2);
send(h.goalPub, msg);
fprintf("Goal sent: (%.2f, %.2f) yaw %.0f deg\n", x, y, yawDeg);

if ~opts.wait, return; end

% Block until OptiTrack says we're within tolerance
t0 = tic;
while true
    p = receive(h.mocapSub, 5);
    d = hypot(p.pose.position.x - x, p.pose.position.y - y);
    if d <= opts.tol
        fprintf("  arrived (%.3f m).\n", d);
        return
    end
    if toc(t0) > opts.timeout
        fprintf("  TIMEOUT at %.3f m.\n", d);
        return
    end
    pause(0.2);
end
end

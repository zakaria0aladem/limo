%% send_route_action.m
% Sends a FIXED waypoint route to Nav2 so every experimental run drives the
% same intended path. Uses the navigate_through_poses action (ordered waypoints).
%
% Run this ONCE PER EXPERIMENTAL RUN, after starting the robot + the stack
% for that run (mocap / amcl / odom-only) and AFTER `ros2 bag record` is rolling.
%
% Requirements: MATLAB R2023a (ROS 2 Foxy), RMW = rmw_fastrtps_cpp, and the
% nav2_msgs interfaces generated ONCE with matlab/setup/gen_nav2_msgs.m.
% No generated messages? Use send_route_goalpose.m instead (zero-dependency).

% ----- Network setup (match the robot) -----
% Domain is passed to ros2node directly: setenv("ROS_DOMAIN_ID") is only read
% once when MATLAB's ROS stack starts, so it is unreliable mid-session.
node = ros2node("/matlab_route_sender", 10);

% ----- DEFINE YOUR ROUTE HERE -----
% Pick reachable points INSIDE your map (read coords by hovering in RViz, or
% drive there and read `tf2_echo map base_link`). Long L + loop, same route as
% send_route_goalpose.m. The robot starts at (0,0) facing +x.
% [x, y, yaw_degrees]
waypoints = [ ...
    2.5,  0.0,   0;    % long straight leg
    2.5,  2.0,  90;    % the L corner (turn)
    0.5,  2.0, 180;    % top leg
    0.5,  0.5, 270;    % heading back down
    0.0,  0.0,   0];   % loop closed -- back to start, SAME heading as origin

% ----- Build the action goal -----
client = ros2actionclient(node,"/navigate_through_poses", ...
                          "nav2_msgs/NavigateThroughPoses");
disp("Waiting for Nav2 action server...");
waitForServer(client);

goalMsg = ros2message(client);
poses = repmat(ros2message("geometry_msgs/PoseStamped"), size(waypoints,1), 1);
for i = 1:size(waypoints,1)
    poses(i).header.frame_id = 'map';
    poses(i).pose.position.x = waypoints(i,1);
    poses(i).pose.position.y = waypoints(i,2);
    yaw = deg2rad(waypoints(i,3));
    poses(i).pose.orientation.z = sin(yaw/2);
    poses(i).pose.orientation.w = cos(yaw/2);
end
goalMsg.poses = poses;

% ----- Send and wait -----
fprintf("Sending %d-waypoint route...\n", size(waypoints,1));
goalHandle = sendGoal(client, goalMsg);
disp("Route sent. Robot is driving. Keep the bag recording until it finishes.");

% Block until Nav2 reports the route finished (or give up after 10 min).
resultMsg = getResult(goalHandle, 600); %#ok<NASGU>
disp("=== Route complete. Stop the bag now (Ctrl+C). ===");

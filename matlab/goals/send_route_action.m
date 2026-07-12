%% send_route.m
% Sends a FIXED waypoint route to Nav2 so every experimental run drives the
% same intended path. Uses the navigate_through_poses action (ordered waypoints).
%
% Run this ONCE PER EXPERIMENTAL RUN, after starting the robot + the stack
% for that run (mocap / amcl / odom-only) and AFTER `ros2 bag record` is rolling.
%
% Requirements: MATLAB R2023a (ROS 2 Foxy), same ROS_DOMAIN_ID + RMW as robot.

% ----- Network setup (match the robot) -----
setenv("ROS_DOMAIN_ID","10");
setenv("RMW_IMPLEMENTATION","rmw_fastrtps_cpp");
node = ros2node("/matlab_route_sender");

% ----- DEFINE YOUR ROUTE HERE -----
% Pick reachable points INSIDE your map (read coords by hovering in RViz, or
% drive there and read `tf2_echo map base_link`). Long L + loop:
% [x, y, yaw_degrees]
waypoints = [ ...
    0.0,  0.0,   0;    % start
    2.5,  0.0,   0;    % long straight leg
    2.5,  2.0,  90;    % the L corner (turn)
    0.5,  2.0, 180;    % top leg
    0.5,  0.5, 270;    % heading back down
    0.0,  0.0, 180];   % loop closed - back to start

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

% Optional: block until done (uncomment to wait for completion)
% resultMsg = getResult(goalHandle);
% disp("Route complete.");

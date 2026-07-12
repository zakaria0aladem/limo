%% send_route_goalpose.m  (v3 -- QoS fixed, uses limo_connect)
% Drives a FIXED waypoint route from MATLAB using /goal_pose
% (geometry_msgs/PoseStamped -- built into MATLAB, no ros2genmsg needed).
% Each goal blocks until OptiTrack says the robot arrived, so waypoints
% sequence automatically.
%
% This is the ZERO-DEPENDENCY route sender. For the action-based version
% (feedback/result/cancel, ordered NavigateThroughPoses) see send_route_action.m,
% which additionally requires gen_nav2_msgs.m to have been run once.
%
% Run once per experimental run, after the stack is up and the bag is rolling.

clear; clc;

h = limo_connect(domainID=10);      % connects + self-tests the mocap subscriber

% ----- ROUTE: [x, y, yaw_degrees] -- set to reachable points in YOUR map -----
waypoints = [ ...
    2.5,  0.0,   0;    % long straight leg
    2.5,  2.0,  90;    % the L corner
    0.5,  2.0, 180;    % top leg
    0.5,  0.5, 270;    % heading back down
    0.0,  0.0,   0];   % loop closed -- back to start, SAME heading as origin

fprintf("\nDriving %d-waypoint route...\n\n", size(waypoints,1));
for i = 1:size(waypoints,1)
    fprintf("Waypoint %d/%d  ", i, size(waypoints,1));
    limo_goal(h, waypoints(i,1), waypoints(i,2), waypoints(i,3), ...
              wait=true, tol=0.20, timeout=90);
end

disp("=== Route complete. Stop the bag now (Ctrl+C). ===");

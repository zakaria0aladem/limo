%% gen_nav2_msgs.m
% ONE-TIME: generate MATLAB interfaces for nav2_msgs (incl. the
% NavigateToPose / NavigateThroughPoses ACTIONS) so ros2actionclient works.
%
% You only need this if you want the ACTION interface (goal feedback, result,
% cancellation). Plain goal-sending via /goal_pose needs NOTHING here --
% geometry_msgs/PoseStamped is built into MATLAB.
%
% WHAT ros2genmsg WANTS
%   A folder containing ROS 2 package folders. Each package has:
%       <pkg>/msg/*.msg      <pkg>/srv/*.srv      <pkg>/action/*.action
%   ros2genmsg reads .action files from the `action` subfolder.
%
% STEP 1 (in the CONTAINER) -- copy nav2_msgs into the mounted folder:
%   cp -r /opt/ros/foxy/share/nav2_msgs /root/maps/custom_msgs/
%   # if the installed share/ lacks the raw .msg/.action sources, clone them:
%   #   git clone -b foxy-devel https://github.com/ros-planning/navigation2
%   #   cp -r navigation2/nav2_msgs /root/maps/custom_msgs/
%   chown -R 1000:1000 /root/maps/custom_msgs     # so MATLAB can write there
%
% STEP 2 -- run this script in MATLAB.
%
% GOTCHAS (from MathWorks Answers, ROS 2 Foxy + Linux):
%   * Start MATLAB from a terminal that has NOT sourced /opt/ros/foxy/setup.bash.
%     A sourced AMENT_PREFIX_PATH makes colcon pick up the system ROS headers
%     and the build fails. Launch MATLAB from a clean shell.
%   * You need a working C++ toolchain (cmake, gcc). Check with `mex -setup C++`.
%   * Use a SHORT folder path; the build is a colcon build and can take minutes.
%   * MATLAB must have WRITE access to the folder (hence the chown above).

folderPath = "/home/zakaria/maps/custom_msgs";   % <-- contains nav2_msgs/

assert(isfolder(folderPath), "Folder not found: %s", folderPath);
fprintf("Generating custom messages from %s\n", folderPath);
fprintf("(this runs a colcon build -- several minutes)\n\n");

ros2genmsg(folderPath)

% ---- Verify ----
fprintf("\nChecking for the action type...\n");
msgs = ros2("msg","list");
hits = msgs(contains(msgs, "NavigateToPose"));
if isempty(hits)
    warning("NavigateToPose not found. Check the build log above.");
else
    disp(hits);
    fprintf("\nSuccess. You can now use:\n");
    fprintf("  [client,goalMsg] = ros2actionclient(node,""/navigate_to_pose"",...\n");
    fprintf("                        ""nav2_msgs/NavigateToPose"");\n");
end

%% ---- Using the action once generated ----
% h = limo_connect(domainID=10);
% [client, goalMsg] = ros2actionclient(h.node, "/navigate_to_pose", ...
%                                      "nav2_msgs/NavigateToPose");
% waitForServer(client);
% goalMsg.pose.header.frame_id = 'map';
% goalMsg.pose.pose.position.x = 1.0;
% goalMsg.pose.pose.position.y = 0.5;
% goalMsg.pose.pose.orientation.w = 1.0;
% gh = sendGoal(client, goalMsg);       % blocks-capable; gives feedback+result
% result = getResult(gh);
%
% The action gives you: distance_remaining, navigation_time, number_of_recoveries
% in feedback, plus the ability to CANCEL a goal -- none of which /goal_pose has.

function h = limo_connect(opts)
%LIMO_CONNECT  Connect MATLAB to the LIMO's ROS 2 network. Returns a handle
%              struct with node, publishers, subscribers, and a TF tree, all
%              created with the CORRECT QoS.
%
%   h = limo_connect();                 % defaults (domain 10)
%   h = limo_connect(domainID=10);
%
% WHY THIS EXISTS
%   1) DOMAIN. Passing domainID to ros2node is deterministic (setenv timing
%      is not -- it's read once when MATLAB's ROS stack initializes).
%   2) QoS. vrpn_mocap and /tf publish BEST-EFFORT. MATLAB defaults to
%      RELIABLE, which cannot match -> receive() times out. We force
%      best-effort on all sensor subscribers and on the TF listener.
%
% PREREQ:
%   * MATLAB Preferences > ROS Toolbox > RMW = rmw_fastrtps_cpp (default)
%   * The Fast DDS UDP profile is active (container/host shared-memory fix):
%       setenv("FASTRTPS_DEFAULT_PROFILES_FILE","/home/zakaria/maps/fastdds_udp.xml")
%     BEFORE launching MATLAB / creating any ros2 object.
%
% Returns h with fields:
%   h.node      ros2node
%   h.goalPub   -> /goal_pose   (geometry_msgs/PoseStamped)
%   h.cmdPub    -> /cmd_vel     (geometry_msgs/Twist)
%   h.mocapSub  -> mocap pose   (best-effort)
%   h.odomSub   -> /odom        (best-effort)
%   h.statusSub -> /limo_status (battery etc.)
%   h.tftree    ros2tf          (for the Nav2 map->base_link estimate)

arguments
    opts.domainID (1,1) double = 10
    opts.nodeName (1,1) string = "/matlab_limo"
    opts.mocapTopic (1,1) string = "/vrpn_mocap/Limo/pose"
    opts.mapFrame (1,1) string = "map"
    opts.baseFrame (1,1) string = "base_link"
    opts.verbose (1,1) logical = true
end

% --- node on the right domain
h.node = ros2node(opts.nodeName, opts.domainID);
h.domainID  = opts.domainID;
h.mocapTopic = opts.mocapTopic;
h.mapFrame  = opts.mapFrame;
h.baseFrame = opts.baseFrame;

% --- publishers: reliable is correct for commands
h.goalPub = ros2publisher(h.node, "/goal_pose", "geometry_msgs/PoseStamped", ...
    "Reliability","reliable","Durability","volatile","Depth",5);
h.cmdPub  = ros2publisher(h.node, "/cmd_vel", "geometry_msgs/Twist", ...
    "Reliability","reliable","Durability","volatile","Depth",5);

% --- subscribers: BEST-EFFORT for high-rate sensor streams
h.mocapSub = ros2subscriber(h.node, opts.mocapTopic, "geometry_msgs/PoseStamped", ...
    "Reliability","besteffort","Durability","volatile","Depth",5);
h.odomSub  = ros2subscriber(h.node, "/odom", "nav_msgs/Odometry", ...
    "Reliability","besteffort","Durability","volatile","Depth",5);

% --- robot status (battery, etc.). Type name can vary; try, warn if absent.
h.statusSub = [];
try
    h.statusSub = ros2subscriber(h.node, "/limo_status", "limo_msgs/LimoStatus", ...
        "Reliability","besteffort","Durability","volatile","Depth",5);
catch
    if opts.verbose
        warning(['/limo_status not subscribed (limo_msgs may not be known to ' ...
                 'MATLAB). Battery will read NaN. This is optional.']);
    end
end

% --- TF tree for the Nav2 estimate (map -> base_link). Best-effort listener.
h.tftree = ros2tf(h.node, ...
    "DynamicListenerQoS", struct('Reliability','besteffort','Depth',100), ...
    "StaticListenerQoS",  struct('Durability','volatile'));

pause(1.5);   % let discovery + TF buffer fill

if opts.verbose
    fprintf("Connected on domain %d.\n", opts.domainID);
    try
        p = receive(h.mocapSub, 5);
        fprintf("  mocap OK: robot at (%.3f, %.3f)\n", ...
            p.pose.position.x, p.pose.position.y);
    catch
        warning("No mocap message. Check vrpn_mocap, topic name, RMW, domain, DDS profile.");
    end
    fprintf("  TF frames seen: %d\n", numel(h.tftree.AvailableFrames));
end
end

%% analyze_runs.m
% Loads the three experimental bags, extracts trajectories, and produces the
% comparison: ground truth (OptiTrack) vs what each run BELIEVED, plus drift.
%
% For each run we extract TWO trajectories from the bag:
%   TRUTH  = /vrpn_mocap/Limo/pose         (where the robot actually was)
%   BELIEF = map->base_link from /tf        (where the robot thought it was)
%
% Mocap & AMCL: belief should hug truth. Odom-only: belief drifts away.
%
% Requirements: MATLAB R2023a+ with ROS Toolbox (ros2bagreader).
% Put the three bag folders next to this script:
%   run1_mocap/  run2_amcl/  run3_odom/

clear; close all; clc;

runs = struct( ...
    'name',  {'Mocap-Nav'}, ...
    'bag',   {'run1_mocap'}, ...
    'color', {[0 0.45 0.74]} );

mocap_topic = "/vrpn_mocap/Limo/pose";
map_frame   = "map";
base_frame  = "base_link";

%% ---- Extract trajectories from each bag ----
for k = 1:numel(runs)
    bag = ros2bagreader(runs(k).bag);

    % TRUTH: OptiTrack pose
    truth = extractPose(bag, mocap_topic);
    runs(k).truth = truth;

    % BELIEF: reconstruct map->base_link from /tf
    runs(k).belief = extractTF(bag, map_frame, base_frame);

    % Drift = distance between belief and truth, time-matched
    runs(k).drift = trajDrift(runs(k).truth, runs(k).belief);
    fprintf("%-10s  truth pts: %4d  belief pts: %4d  final drift: %.3f m  max drift: %.3f m\n", ...
        runs(k).name, size(truth.xy,1), size(runs(k).belief.xy,1), ...
        runs(k).drift.final, runs(k).drift.max);
end

%% ---- FIGURE 1: overlaid paths (the money plot) ----
figure('Color','w','Position',[100 100 900 700]); hold on; grid on; axis equal;
% Ground truth: same physical path each run; plot run-1 truth as THE real path
plot(runs(1).truth.xy(:,1), runs(1).truth.xy(:,2), 'k-', 'LineWidth', 2.5, ...
     'DisplayName','Real path (OptiTrack truth)');
for k = 1:numel(runs)
    plot(runs(k).belief.xy(:,1), runs(k).belief.xy(:,2), '--', ...
         'Color', runs(k).color, 'LineWidth', 1.8, ...
         'DisplayName', sprintf('%s (believed)', runs(k).name));
end
xlabel('x [m]'); ylabel('y [m]');
title('Trajectory comparison: believed path vs ground truth');
legend('Location','best'); set(gca,'FontSize',12);

%% ---- FIGURE 2: drift over time ----
figure('Color','w','Position',[100 100 900 400]); hold on; grid on;
for k = 1:numel(runs)
    plot(runs(k).drift.t, runs(k).drift.d, '-', ...
         'Color', runs(k).color, 'LineWidth', 1.8, 'DisplayName', runs(k).name);
end
xlabel('time [s]'); ylabel('position error vs truth [m]');
title('Localization drift over time (belief - truth)');
legend('Location','northwest'); set(gca,'FontSize',12);

%% ---- Summary table ----
fprintf('\n=== Summary (error of believed position vs OptiTrack truth) ===\n');
fprintf('%-12s %10s %10s %10s\n','Run','RMSE[m]','Max[m]','Final[m]');
for k = 1:numel(runs)
    fprintf('%-12s %10.3f %10.3f %10.3f\n', runs(k).name, ...
        runs(k).drift.rmse, runs(k).drift.max, runs(k).drift.final);
end

%% ================= helper functions =================
function P = extractPose(bag, topic)
    sel = select(bag, 'Topic', topic);
    msgs = readMessages(sel);
    n = numel(msgs);
    xy = zeros(n,2); t = zeros(n,1);
    for i = 1:n
        xy(i,:) = [msgs{i}.pose.position.x, msgs{i}.pose.position.y];
        t(i) = double(msgs{i}.header.stamp.sec) + double(msgs{i}.header.stamp.nanosec)*1e-9;
    end
    P.xy = xy; P.t = t - t(1);
end

function B = extractTF(bag, parent, child)
    % Reconstruct parent->child by composing map->odom and odom->base_link
    % from /tf. Assumes a 2-hop chain (map->odom->base_link).
    sel = select(bag, 'Topic', '/tf');
    msgs = readMessages(sel);
    mo = []; ob = [];   % [t x y yaw]
    for i = 1:numel(msgs)
        for j = 1:numel(msgs{i}.transforms)
            tr = msgs{i}.transforms(j);
            t = double(tr.header.stamp.sec) + double(tr.header.stamp.nanosec)*1e-9;
            x = tr.transform.translation.x; y = tr.transform.translation.y;
            q = tr.transform.rotation; yaw = atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y^2+q.z^2));
            if strcmp(tr.header.frame_id, parent) && strcmp(tr.child_frame_id, 'odom')
                mo = [mo; t x y yaw]; %#ok<AGROW>
            elseif strcmp(tr.header.frame_id, 'odom') && strcmp(tr.child_frame_id, child)
                ob = [ob; t x y yaw]; %#ok<AGROW>
            end
        end
    end
    % Compose at each odom->base sample, using nearest map->odom in time
    xy = zeros(size(ob,1),2); tt = zeros(size(ob,1),1);
    for i = 1:size(ob,1)
        if isempty(mo)
            m = [0 0 0];                       % no map->odom (e.g. pure odom run w/o it)
        else
            [~,idx] = min(abs(mo(:,1)-ob(i,1)));
            m = mo(idx,2:4);
        end
        % 2D compose: map->base = (map->odom) . (odom->base)
        c = cos(m(3)); s = sin(m(3));
        xy(i,1) = m(1) + c*ob(i,2) - s*ob(i,3);
        xy(i,2) = m(2) + s*ob(i,2) + c*ob(i,3);
        tt(i) = ob(i,1);
    end
    B.xy = xy; B.t = tt - tt(1);
end

function D = trajDrift(truth, belief)
    % Distance between belief and truth, matched by nearest timestamp.
    n = size(belief.xy,1);
    d = zeros(n,1);
    for i = 1:n
        [~,idx] = min(abs(truth.t - belief.t(i)));
        d(i) = hypot(belief.xy(i,1)-truth.xy(idx,1), belief.xy(i,2)-truth.xy(idx,2));
    end
    D.t = belief.t; D.d = d;
    D.rmse = sqrt(mean(d.^2));
    D.max = max(d);
    D.final = d(end);
end

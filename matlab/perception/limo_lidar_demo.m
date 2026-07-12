%% limo_lidar_demo.m
% Live LiDAR obstacle viewer. Shows the scan as a polar plot, marks the nearest
% obstacle, and colors the front/left/right sectors green (clear) or red (blocked).
%
% Run:
%   h = limo_connect(domainID=10);
%   limo_lidar_demo          % Ctrl+C in the Command Window to stop
%
% Pure MATLAB, reads /scan directly. Uses limo_lidar_obstacle for the logic.

if ~exist('h','var')
    h = limo_connect(domainID=10);
end

threshold = 0.5;    % [m] obstacle distance
runTime   = 120;    % [s] how long to run

fig = figure('Name','LIMO LiDAR','Color','w','Position',[200 200 700 700]);
ax = polaraxes(fig); hold(ax,'on');
ax.ThetaZeroLocation = 'top';    % 0 deg = front = up
ax.ThetaDir = 'counterclockwise';

fprintf('LiDAR demo running for %d s. Ctrl+C to stop.\n', runTime);
t0 = tic;
while ishandle(fig) && toc(t0) < runTime
    obs = limo_lidar_obstacle(h, threshold=threshold, print=false);
    if ~obs.ok, pause(0.1); continue; end

    cla(ax);
    % all scan points
    polarplot(ax, obs.scan.angles, obs.scan.ranges, '.', ...
        'Color',[0.4 0.4 0.4], 'MarkerSize',4);

    % nearest obstacle marker
    if isfinite(obs.distance)
        c = [0.2 0.6 0.2]; if obs.is_obstacle, c = [0.85 0.15 0.15]; end
        polarplot(ax, deg2rad(obs.angle_deg), obs.distance, 'o', ...
            'MarkerFaceColor',c,'MarkerEdgeColor','k','MarkerSize',12);
    end

    % sector status in the title
    title(ax, sprintf(['nearest %.2f m @ %+.0f deg   |   ' ...
        'F:%s  L:%s  R:%s'], obs.distance, obs.angle_deg, ...
        cs(obs.front.clear), cs(obs.left.clear), cs(obs.right.clear)));
    ax.RLim = [0 4];
    drawnow;
    pause(0.05);
end
fprintf('LiDAR demo stopped.\n');

function s = cs(clear)
if clear, s = 'clear'; else, s = 'BLOCK'; end
end

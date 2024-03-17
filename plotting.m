classdef plotting
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    

    methods(Static)


        function plot_marker_3d(pts_3d, frames, fitted_pts_3d, fig_title)
            if nargin < 2
                frames = 1:size(pts_3d, 1);
            end

            if nargin < 4
                fig_title = '3D points';
            end

            num_axes = size(pts_3d, 2);

            frames = frames + 1; % frames are 1-based indices

            figure('Name', fig_title);
            for ax = 1:num_axes
                subplot(1, num_axes, ax);
                plot(frames, pts_3d(:, ax), 'o-', 'MarkerSize', 2);
                xlabel('Frames');
                ylabel('Position (m)');
                title(char('X' + ax - 1) + ' Axis');
            end

            if nargin >= 3 && ~isempty(fitted_pts_3d)
                for ax = 1:num_axes
                    subplot(1, num_axes, ax);
                    hold on;
                    plot(frames, fitted_pts_3d(:, ax));
                    err = pts_3d(:, ax) - fitted_pts_3d(:, ax);
                    pad = 2 * std(err); % 95%
                    ylim([min(fitted_pts_3d(:, ax)) - pad, max(fitted_pts_3d(:, ax)) + pad]);
                    legend('Original', 'Curve Fit');
                    hold off;
                end
            end
        end


    end
end
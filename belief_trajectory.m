%% Belief trajectory

clc; clear; close;

% Load as tables to access columns by name
states   = readtable('solution_barrier_rrt.csv');

% Extract state columns
x           = states.x;
y           = states.y;
sigma_trace = states.sigma_trace;
lambda      = states.lambda_trace;

figure; hold on;

% --- Interpolation parameter ---
t = 1:length(x);
tt = linspace(1, length(x), 200);   % dense sampling for smoothness

% --- Smooth trajectory with spline ---
xx = spline(t, x, tt);
yy = spline(t, y, tt);

% --- Plot uncertainty tube (sigma_trace circles at waypoints) ---
theta = linspace(0, 2*pi, 50);
for i = 1:length(x)
    tubeX = x(i) + sigma_trace(i) * cos(theta);
    tubeY = y(i) + sigma_trace(i) * sin(theta);
    fill(tubeX, tubeY, [1 0.6 0.6], 'EdgeColor', 'none', 'FaceAlpha', 0.3);
end

% --- Plot smooth trajectory ---
plot(xx, yy, 'b-', 'LineWidth', 2);        % smooth blue line
plot(x, y, 'bo', 'MarkerSize', 2, 'MarkerFaceColor', 'b'); % waypoints

% --- Highlight start location ---
plot(xx(1),yy(1), 'k*', 'MarkerSize', 10, 'MarkerFaceColor', 'k'); % green square
text(round(xx(1))-3, round(yy(1))+3, 'Start', 'Color', 'k', 'FontSize', 12, 'FontWeight', 'bold');

% --- Highlight goal ---
plot(xx(end), yy(end), 'r*', 'MarkerSize', 12, 'LineWidth', 2);
text(round(xx(end))+1, round(yy(end))+1, 'Goal', 'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold');

% --- Obstacles ---
% ob1 = [45 35; 50 35; 50 40; 45 40];
% ob2 = [35 35; 40 35; 40 40; 35 40];
% fill(ob1(:,1), ob1(:,2), [0.8 0 0], 'FaceAlpha', 0.5, 'EdgeColor', 'k');
% fill(ob2(:,1), ob2(:,2), [0.8 0 0], 'FaceAlpha', 0.5, 'EdgeColor', 'k');

% --- Rectangular obstacle (approximation of circle at 25,25 with radius 10) ---
% This matches the 2d_circle_approximation.yaml scene file
% obstacle_x = [15, 35, 35, 15, 15];  % x coordinates of rectangle
% obstacle_y = [15, 15, 35, 35, 15];  % y coordinates of rectangle
obstacle_x = [25, 35, 35, 25, 25];  % x coordinates of rectangle
obstacle_y = [25, 25, 35, 35, 25];  % y coordinates of rectangle

% 1st obstacle
obstacle1_x = [25, 35, 35, 25, 25];
obstacle1_y = [25, 25, 35, 35, 25];

% 2nd obstacle
obstacle2_x = [25, 35, 35, 25, 25];
obstacle2_y = [ 5,  5, 15, 15,  5];

fill(obstacle1_x, obstacle1_y, [0.8 0 0], 'FaceAlpha', 0.5, 'EdgeColor', 'k');
fill(obstacle2_x, obstacle2_y, [0.8 0 0], 'FaceAlpha', 0.5, 'EdgeColor', 'k');

% --- Styling ---
xlabel('x');
ylabel('y');
grid on;
axis equal;

exportgraphics(gcf, 'trajectory.png', 'Resolution', 300);
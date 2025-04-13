clear; close all; clc;
addpath('../')

beam_size = [1; 0.1; 0.5];

beam1_mass = 1;
beam1 = RigidBody(beam1_mass, get_box_inertia(beam1_mass, beam_size));
beam1.setPosition([0.5, 0, 0]);
beam1.setRotation(pi/2, [0, 0, 0]);

beam2_mass = 100;
beam2 = RigidBody(beam2_mass, get_box_inertia(beam2_mass, beam_size));
beam2.setPosition([1, 0.5, 0]);
beam2.setRotation(pi/2, [0, 0, 1]);

%% Main loop
dt = 1e-3; % consider decreasing with unfavourable mass and inertia ratios
t = 0:dt:10;
gif_name = "gif/double_pendulum_"...
    + string(datetime('now','Format','d_MMM_y_HH_mm_ss')) + ".gif";
figure('Position', [10 10 900 600])
for k = 1:length(t)
    % Set gravity
    beam1.addForce([0; 0; -9.81*beam1.m], beam1.x);
    beam2.addForce([0; 0; -9.81*beam2.m], beam2.x);
    % Project
    beam1.update(dt);
    beam2.update(dt);
    % Solve the constraints
    lambda = zeros(6,1);
    for iter = 1:20 % consider increasing with unfavourable mass and inertia ratios
        lambda(1) = beam1.fixPoint([-0.5,0,0], [0,0,0], 0, lambda(1), dt);
        lambda(2) = beam1.linkPoints(beam2, [0.5,0,0], [-0.5,0,0], 0, lambda(2), dt);
        lambda(3) = beam1.linkAxes(beam2, [0,0,1], [0,0,1], 0, lambda(3), dt);
        %lambda(4) = beam1.linkAxes(beam2, [1,0,0], [0,-1,0], 0, lambda(4), dt);
        %lambda(5) = beam2.fixAxis([1,0,0], [0,1,0], 0, lambda(5), dt);
        %lambda(6) = beam1.fixAxis([1,0,0], [1,0,0], 0, lambda(6), dt);
    end
    % Show animation
    if (~mod(k, 100))
        clf;
        show_box(beam1.x, beam1.R, beam_size, 'r')
        show_box(beam2.x, beam2.R, beam_size, 'g')
        hold on
        plot3(0,0,0,'k.','MarkerSize',10)
        x = [beam1.x + beam1.R*[-0.5;0;0], ...
            beam1.x + beam1.R*[0.5;0;0], ...
            beam2.x + beam2.R*[-0.5;0;0]];
        plot3(x(1,:),x(2,:),x(3,:),'k.','MarkerSize',10)
        plot3(0,0,0,'k.','MarkerSize',10)
        title(sprintf("Time: %4.1f s", t(k)))
        view(30,30)
        xlim([-2.5,2.5])
        ylim([-2.5,2.5])
        zlim([-2.5,1])
        drawnow;
        exportgraphics(gcf,gif_name,'Append',true);
    end
end

%% Functions
function inertia = get_box_inertia(box_mass, box_size)
    inertia = diag(box_mass/12*[
        box_size(2)^2 + box_size(3)^2
        box_size(1)^2 + box_size(3)^2
        box_size(1)^2 + box_size(2)^2]);
end

function show_box(x, R, siz, col)
    coord = [
        -1 -1 -1;
        +1 -1 -1;
        +1 +1 -1;
        -1 +1 -1;
        -1 -1 +1;
        +1 -1 +1;
        +1 +1 +1;
        -1 +1 +1] .* siz(:)'*0.5;
    coord = (R*coord')' + x(:)';
    idx = [
        4 8 5 1 4
        1 5 6 2 1
        2 6 7 3 2
        3 7 8 4 3
        5 8 7 6 5
        1 4 3 2 1];
    patch('vertices', coord, 'faces', idx, ...
        'facecolor', col, 'facealpha', 1, 'EdgeColor', 'none');
    hold on
    quiver3(x(1),x(2),x(3),R(1,1),R(2,1),R(3,1),'r','LineWidth',2)
    quiver3(x(1),x(2),x(3),R(1,2),R(2,2),R(3,2),'g','LineWidth',2)
    quiver3(x(1),x(2),x(3),R(1,3),R(2,3),R(3,3),'b','LineWidth',2)
    light("Style","local","Position",[-10 -50 100]);
    xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
    box on; grid on; axis equal
end
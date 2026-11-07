function show_box(x, R, siz, col, show_frame)
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
    if show_frame
        quiver3(x(1),x(2),x(3),R(1,1),R(2,1),R(3,1),'r','LineWidth',2)
        quiver3(x(1),x(2),x(3),R(1,2),R(2,2),R(3,2),'g','LineWidth',2)
        quiver3(x(1),x(2),x(3),R(1,3),R(2,3),R(3,3),'b','LineWidth',2)
    end
    light("Style","local","Position",[-10 -50 100]);
    xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
    box on; grid on; axis equal
end
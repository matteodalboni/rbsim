function draw_scene(rb, cont, body_size, F, x, stride, X, Y, Z)
    show_box(rb.x, rb.R, body_size); hold on
    plot3(x(1,:),x(2,:),x(3,:),'k.','MarkerSize',25)
    flt = 1:stride:size(x,2);
    plot3(x(1,flt),x(2,flt),x(3,flt),'.','MarkerSize',20,'Color','#808080')
    quiver3(rb.x(1),rb.x(2),rb.x(3),rb.R(1,1),rb.R(2,1),rb.R(3,1),'r','LineWidth',2)
    quiver3(rb.x(1),rb.x(2),rb.x(3),rb.R(1,2),rb.R(2,2),rb.R(3,2),'g','LineWidth',2)
    quiver3(rb.x(1),rb.x(2),rb.x(3),rb.R(1,3),rb.R(2,3),rb.R(3,3),'b','LineWidth',2)
    quiver3(cont.x0(1,:),cont.x0(2,:),cont.x0(3,:),F(1,:),F(2,:),F(3,:),'r','LineWidth',1.5)
    surf(X,Y,Z,'FaceColor','#C6A664','EdgeColor','w')
end

function show_box(x, R, siz)
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
        'facecolor', '#77AC30', 'facealpha', 1, 'EdgeColor', 'none');
    light("Style","local","Position",[-10 -50 100]);
    xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
    box on; grid on; axis equal
    xlim([-2, 2]+x(1))
    ylim([-2, 2]+x(2))
    zlim([-1, 1]+x(3))
end
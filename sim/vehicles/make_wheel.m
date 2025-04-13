function wheel = make_wheel(n_wheel_pts, n_layers, radius, width)
    theta = linspace(0, 2*pi, n_wheel_pts + 1); theta(end) = [];
    wheel.n_layers = n_layers;
    wheel.radius = radius;
    wheel.width = width;
    wheel.theta = theta;
    wheel.w = 0;
    xl = radius*[cos(theta); zeros(size(theta)); sin(theta)];
    x0 = [
        zeros(1, wheel.n_layers);
        linspace(-wheel.width/2, wheel.width/2, wheel.n_layers);
        zeros(1, wheel.n_layers)
        ];
    wheel.xl = clone_points(xl, x0);
    wheel.vl = cross(repmat([0; wheel.w; 0],1,size(wheel.xl,2)), wheel.xl);
end
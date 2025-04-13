function wheel = update_wheel(wheel, dt)
    theta = wheel.theta - wheel.w*dt;
    xl = wheel.radius*[cos(theta); zeros(size(theta)); sin(theta)];
    wheel.theta = theta;
    x0 = [
        zeros(1, wheel.n_layers);
        linspace(-wheel.width/2, wheel.width/2, wheel.n_layers);
        zeros(1, wheel.n_layers)
        ];
    wheel.xl = clone_points(xl, x0);
    wheel.vl = cross(repmat([0; wheel.w; 0],1,size(wheel.xl,2)), wheel.xl);
end
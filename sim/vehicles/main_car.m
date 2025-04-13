% The car is modeled as a single rigid body. Wheel dynamics are neglected. 
clear; close all; clc;
addpath('../')

body_mass = 20;
body_size = [1; 0.6; 0.4];
body_inertia = diag(body_mass/12*[
    body_size(2)^2 + body_size(3)^2
    body_size(1)^2 + body_size(3)^2
    body_size(1)^2 + body_size(2)^2]);
rb = RigidBody(body_mass, body_inertia);
rb.setPosition([0, 0, 0.8]);
%rb.setRotation(pi/6, [0, 0, 1]);

n_layer_pts = 30;
n_layers = 3;
radius = 0.25;
width = 0.15;
[soil, X, Y, Z] = generate_soil();
cont = Contact(4*n_layer_pts*n_layers, soil, 400, 3, 0.2, 0.85);

wheel_l = make_wheel(n_layer_pts, n_layers, radius, width);
wheel_r = make_wheel(n_layer_pts, n_layers, radius, width);

%% Main loop
dt = 1e-3;
t = 0:dt:12;
hfig = figure;
set(hfig,'KeyPressFcn',@set_car_input_by_key);
setappdata(hfig, 'wl', 0);
setappdata(hfig, 'wr', 0);
setappdata(hfig, 'alpha', 0);
setappdata(hfig, 'az', -30);
setappdata(hfig, 'el', 20);
k = 0;
while 1
    k = k + 1;
    % Computation of forces and torques
    rb.addForce([0; 0; -9.81*rb.m], rb.x);
    wheel_l.w = getappdata(hfig, 'wl');
    wheel_r.w = getappdata(hfig, 'wr');
    alpha = getappdata(hfig, 'alpha');
    q = [cos(0.5*alpha), sin(0.5*alpha)*[0,0,1]];
    q = q / norm(q);
    track_xl = [
        quat2rotm(q)*(wheel_l.xl) + [+body_size(1)/2; +0.4; -0.4], ...
        wheel_l.xl + [-body_size(1)/2; +0.4; -0.4], ...
        quat2rotm(q)*(wheel_r.xl) + [+body_size(1)/2; -0.4; -0.4], ...
        wheel_r.xl + [-body_size(1)/2; -0.4; -0.4]
        ];
    track_vl = [
        rb.R*quat2rotm(q)*wheel_l.vl, ...
        rb.R*wheel_l.vl, ...
        rb.R*quat2rotm(q)*wheel_r.vl, ...
        rb.R*wheel_r.vl];
    x = rb.x + rb.R*track_xl;
    v = rb.getVelocity(x) + track_vl;
    F = cont.getForces(x, v);
    rb.addForce(F, cont.x0);
    % Vehicle update
    wheel_l = update_wheel(wheel_l, dt);
    wheel_r = update_wheel(wheel_r, dt);
    rb.update(dt);
    % Animation
    if (~mod(k, 100))
        clf;
        draw_scene(rb, cont, body_size, F, x, n_layer_pts, X, Y, Z)
        view(getappdata(hfig, 'az'), getappdata(hfig, 'el'))
        title(sprintf("wl:%+5d rad/s | wr:%+5d rad/s",wheel_l.w,wheel_r.w))
        drawnow;
    end
end
classdef RigidBody < handle
    properties
        m = 0 % mass
        Ib = zeros(3) % inertia tensor in the body frame
        Ib_inv = zeros(3) % inverse of inertia tensor in the body frame
        x = zeros(3,1) % position of the center of mass
        q = [1;0;0;0] % rotation quaternion
        P = zeros(3,1) % linear momentum
        L = zeros(3,1) % angular momentum
        R = zeros(3) % rotation matrix
        v = zeros(3,1) % velocity of the center of mass
        w = zeros(3,1) % angular velocity
        F = zeros(3,1) % total force
        T = zeros(3,1) % total torque
    end

    methods
        function rb = RigidBody(m, Ib)
            rb.m = m;
            rb.Ib = Ib;
            rb.Ib_inv = inv(Ib);
        end

        function setPosition(rb, x)
            rb.x = x(:);
        end

        function setVelocity(rb, v)
            rb.v = v(:);
        end

        function setRotation(rb, alpha, direction)
            rb.q = [cos(0.5*alpha); sin(0.5*alpha)*direction(:)];
            rb.q = rb.q / norm(rb.q);
            rb.R = quat2rotm(rb.q');
        end

        function vq = getVelocity(rb, xq)
            % Each column of xq is the position of one query point
            vq = rb.v + cross(rb.w.*ones(size(xq)), xq - rb.x);
        end

        function addTorque(rb, T)
            % Each column of T is a torque vector
            rb.T = rb.T + sum(T, 2);
        end

        function addForce(rb, F, xa)
            % Each column of F is a force vector, each column of xa is the
            % position of one point of application
            r = xa - rb.x;
            rb.F = rb.F + sum(F, 2);
            rb.T = rb.T + sum(cross(r, F), 2);
        end

        function update(rb, dt)
            % Time integration by fourth order Runge-Kutta method
            y = rb.getStates();
            s1 = rb.getStatesDot();
            rb.setStates(y + dt*s1*0.5);
            s2 = rb.getStatesDot();
            rb.setStates(y + dt*s2*0.5);
            s3 = rb.getStatesDot();
            rb.setStates(y + dt*s3);
            s4 = rb.getStatesDot();
            rb.setStates(y + dt*(s1 + 2*s2 + 2*s3 + s4)/6);
            % Update of the auxiliary variables
            rb.q = rb.q / norm(rb.q);
            rb.R = quat2rotm(rb.q');
            rb.v = rb.P / rb.m;
            rb.w = rb.R * rb.Ib_inv * rb.R' * rb.L;
            % Reset of forces and torques
            rb.F = zeros(3,1);
            rb.T = zeros(3,1);
        end

        function lambda = linkPoints(rb1, rb2, xb1, xb2, compl, lambda, dt)
            % Link the points xb1 and xb2 to each other. The points are
            % expressed in the body reference frames and belong to this
            % rigid body (rb1) and to the rigid body rb2, respectively.
            r1 = rb1.x + rb1.R*xb1(:);
            r2 = rb2.x + rb2.R*xb2(:);
            g = r1 - r2;
            c = norm(g);
            n = g; % / max(eps,c);
            I_inv1 = rb1.R * rb1.Ib_inv * rb1.R';
            I_inv2 = rb2.R * rb2.Ib_inv * rb2.R';
            c1 = cross(r1,n);
            c2 = cross(r2,n);
            w1 = 1/rb1.m + c1'*I_inv1*c1;
            w2 = 1/rb2.m + c2'*I_inv2*c2;
            dlambda = -(c + compl/dt^2*lambda)/(w1 + w2 + compl/dt^2);
            lambda = lambda + dlambda;
            p = dlambda*n;
            rb1.x = rb1.x + p/rb1.m;
            rb2.x = rb2.x - p/rb2.m;
            rb1.q = rb1.q + 0.5*quatmultiply([0;I_inv1*cross(r1,p)]',rb1.q')';
            rb2.q = rb2.q - 0.5*quatmultiply([0;I_inv2*cross(r2,p)]',rb2.q')';
            rb1.q = rb1.q / norm(rb1.q);
            rb2.q = rb2.q / norm(rb2.q);
            rb1.R = quat2rotm(rb1.q');
            rb2.R = quat2rotm(rb2.q');
            f = dlambda*n/dt^2;
            rb1.P = rb1.P + f*dt;
            rb2.P = rb2.P - f*dt;
            rb1.L = rb1.L + cross(r1 - rb1.x, f)*dt;
            rb2.L = rb2.L - cross(r2 - rb2.x, f)*dt;
            rb1.v = rb1.P / rb1.m;
            rb2.v = rb2.P / rb2.m;
            rb1.w = rb1.R * rb1.Ib_inv * rb1.R' * rb1.L;
            rb2.w = rb2.R * rb2.Ib_inv * rb2.R' * rb2.L;
        end

        function lambda = linkAxes(rb1, rb2, axb1, axb2, compl, lambda, dt)
            % Link the axes axb1 and axb2 to each other. The axes are
            % expressed in the body reference frames and belong to this
            % rigid body (rb1) and to the rigid body rb2, respectively.
            ax1 = rb1.R*axb1(:);
            ax2 = rb2.R*axb2(:);
            ax1 = ax1 / norm(ax1);
            ax2 = ax2 / norm(ax2);
            g = cross(ax2, ax1);
            c = norm(g);
            n = g / max(eps,c);
            I_inv1 = rb1.R * rb1.Ib_inv * rb1.R';
            I_inv2 = rb2.R * rb2.Ib_inv * rb2.R';
            w1 = n'*I_inv1*n;
            w2 = n'*I_inv2*n;
            dlambda = -(c + compl/dt^2*lambda)/(w1 + w2 + compl/dt^2 + eps);
            lambda = lambda + dlambda;
            p = dlambda*n;
            rb1.q = rb1.q + 0.5*quatmultiply([0;I_inv1*p]',rb1.q')';
            rb2.q = rb2.q - 0.5*quatmultiply([0;I_inv2*p]',rb2.q')';
            rb1.q = rb1.q / norm(rb1.q);
            rb2.q = rb2.q / norm(rb2.q);
            rb1.R = quat2rotm(rb1.q');
            rb2.R = quat2rotm(rb2.q');
            t = dlambda*n/dt^2;
            rb1.L = rb1.L + t*dt;
            rb2.L = rb2.L - t*dt;
            rb1.w = rb1.R * rb1.Ib_inv * rb1.R' * rb1.L;
            rb2.w = rb2.R * rb2.Ib_inv * rb2.R' * rb2.L;
        end

        function lambda = fixPoint(rb1, xb1, x2, compl, lambda, dt)
            % Fix the point xb1 to the point x2. xb1 is expressed in the
            % body reference frame and belongs to this rigid body (rb1),
            % whereas x2 is expressed in the world reference frame.
            r1 = rb1.x + rb1.R*xb1(:);
            g = r1 - x2(:);
            c = norm(g);
            n = g; % / max(eps,c);
            I_inv1 = rb1.R * rb1.Ib_inv * rb1.R';
            c1 = cross(r1,n);
            w1 = 1/rb1.m + c1'*I_inv1*c1;
            dlambda = -(c + compl/dt^2*lambda)/(w1 + compl/dt^2);
            lambda = lambda + dlambda;
            p = dlambda*n;
            rb1.x = rb1.x + p/rb1.m;
            rb1.q = rb1.q + 0.5*quatmultiply([0;I_inv1*cross(r1,p)]',rb1.q')';
            rb1.q = rb1.q / norm(rb1.q);
            rb1.R = quat2rotm(rb1.q');
            f = dlambda*n/dt^2;
            rb1.P = rb1.P + f*dt;
            rb1.L = rb1.L + cross(r1 - rb1.x, f)*dt;
            rb1.v = rb1.P / rb1.m;
            rb1.w = rb1.R * rb1.Ib_inv * rb1.R' * rb1.L;
        end

        function lambda = fixAxis(rb1, axb1, ax2, compl, lambda, dt)
            % Fix the axis axb1 to the axis ax2. axb1 is expressed in the
            % body reference frame and belongs to this rigid body (rb1),
            % whereas ax2 is expressed in the world reference frame.
            ax1 = rb1.R*axb1(:);
            ax1 = ax1 / norm(ax1);
            ax2 = ax2(:) / norm(ax2);
            g = cross(ax2, ax1);
            c = norm(g);
            n = g / max(eps,c);
            I_inv1 = rb1.R * rb1.Ib_inv * rb1.R';
            w1 = n'*I_inv1*n;
            dlambda = -(c + compl/dt^2*lambda)/(w1 + compl/dt^2 + eps);
            lambda = lambda + dlambda;
            p = dlambda*n;
            rb1.q = rb1.q + 0.5*quatmultiply([0;I_inv1*p]',rb1.q')';
            rb1.q = rb1.q / norm(rb1.q);
            rb1.R = quat2rotm(rb1.q');
            t = dlambda*n/dt^2;
            rb1.L = rb1.L + t*dt;
            rb1.w = rb1.R * rb1.Ib_inv * rb1.R' * rb1.L;
        end
    end

    methods (Access = private)
        function setStates(rb, y)
            rb.x(:) = y( 1: 3);
            rb.q(:) = y( 4: 7);
            rb.P(:) = y( 8:10);
            rb.L(:) = y(11:13);
        end

        function y = getStates(rb)
            y = zeros(13,1);
            y( 1: 3) = rb.x(:);
            y( 4: 7) = rb.q(:);
            y( 8:10) = rb.P(:);
            y(11:13) = rb.L(:);
        end

        function ydot = getStatesDot(rb)
            q_ = rb.q / norm(rb.q);
            R_ = quat2rotm(q_');
            v_ = rb.P / rb.m;
            w_ = R_ * rb.Ib_inv * R_' * rb.L;
            ydot = zeros(13,1);
            ydot( 1: 3) = v_;
            ydot( 4: 7) = 0.5 * quatmultiply([0, w_'], q_')';
            ydot( 8:10) = rb.F;
            ydot(11:13) = rb.T;
        end
    end
end
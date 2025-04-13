classdef Contact < handle
    properties
        x0 % points of contact
        on % filter for active contact points
        soil % soil function
        k % stiffness
        dn % normal damping
        dt % tangential damping
        mu % coefficient of static friction
        delta % displacement for the computation of the normal versor
    end

    methods
        function cont = Contact(num_contact_points, soil, k, dn, dt, mu)
            cont.on = false(1,num_contact_points);
            cont.x0 = zeros(3,length(cont.on));
            cont.soil = soil;
            cont.k = k;
            cont.dn = dn;
            cont.dt = dt;
            cont.mu = mu;
            cont.delta = 0.01;
        end

        function F = getForces(cont, x, v)
            h = cont.soil(x(1,:), x(2,:));
            flt = ~cont.on & (x(3,:) < h);
            cont.on(flt) = true;
            cont.x0(1:2,flt) = x(1:2,flt);
            cont.x0(3,flt) = h(flt);
            cont.on(x(3,:) >= h) = false;
            %
            dx =  [
                repmat([cont.delta; 0], 1, length(h(cont.on)))
                cont.soil(x(1,cont.on) + cont.delta, x(2,cont.on)) - h(cont.on)
                ];
            dy =  [
                repmat([0; cont.delta], 1, length(h(cont.on)))
                cont.soil(x(1,cont.on), x(2,cont.on) + cont.delta) - h(cont.on)
                ];
            n = cross(dx, dy);
            n = n./max(eps, vecnorm(n, 2, 1));
            vn = dot(v(:,cont.on), n).*n;
            vt = v(:,cont.on) - vn;
            F = zeros(size(x));
            F0 = cont.k*(cont.x0(:,cont.on) - x(:,cont.on)) - cont.dn*vn - cont.dt*vt;
            F0 = F0 - min(dot(F0, n), 0).*n;
            Fn = dot(F0, n).*n;
            Ft0 = F0 - Fn;
            Ft0_mag = vecnorm(Ft0, 2, 1);
            Ft_mag_max = vecnorm(cont.mu*Fn, 2, 1);
            Ft_mag = min(Ft0_mag, Ft_mag_max);
            Ft = Ft_mag.*Ft0./max(eps, Ft0_mag);
            F(:,cont.on) = Fn + Ft;
            idx = find(cont.on);
            cont.on(idx(Ft_mag == Ft_mag_max)) = false;
        end
    end
end
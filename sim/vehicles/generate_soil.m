function [soil, X, Y, Z] = generate_soil()
    x = -10:0.1:10;
    y = -5:0.1:5;
    [X,Y] = meshgrid(x,y);
    %Z = 0.5*(X.*exp(-0.001*X.^2-0.001*Y.^2));
    Z = min(max(0, 0.5*(X - 2)), 2) + 0.2*rand(size(X));
    soil = griddedInterpolant(X',Y',Z');
end
function xl = clone_points(xl, coord)
    P = cell(1,size(coord,2));
    for k = 1:length(P)
        P{k} = xl + coord(:,k);
    end
    xl = cat(2, P{:});
end
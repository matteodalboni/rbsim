function inertia = get_box_inertia(box_mass, box_size)
    inertia = diag(box_mass/12*[
        box_size(2)^2 + box_size(3)^2
        box_size(1)^2 + box_size(3)^2
        box_size(1)^2 + box_size(2)^2]);
end
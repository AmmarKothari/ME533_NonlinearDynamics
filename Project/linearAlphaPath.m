function alpha_points = linearAlphaPath(start_alphas, goal_alphas, pts)
% creates an array that is effectively a linspace between two vectors
    s = length(start_alphas);
    alpha_points = zeros(pts, s);
    for i = 1:s
        alpha_points(:, i) = linspace(start_alphas(i), goal_alphas(i), pts);
    end
end
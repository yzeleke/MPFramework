% Calcular the heuristic distance from start vertex, start_v 
% to current vertex, curr_v
function h = calc_heuristic(start_v, curr_v)

    % Pythagorean heuristic...could be different...e.g: account for other
    % states like theta when doing dubins?
    x_dist = abs(start_v.x - curr_v.x) + 1;
    y_dist = abs(start_v.y - curr_v.y) + 1;
    h = sqrt(x_dist^2 + y_dist^2);
end
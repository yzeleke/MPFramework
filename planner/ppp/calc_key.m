% Calculate the priority queue key
function key = calc_key(vertex, start_v)
    h = calc_heuristic(start_v, vertex);
    key = min([vertex.g, vertex.rhs + h]); % Affecting the heuristic affects the path finding
end
% Calculate the priority queue key
function key = calc_key(vertex, start_v)
    h = calc_heuristic(start_v, vertex);
    key = min([vertex.g, vertex.rhs + h*2]);
end
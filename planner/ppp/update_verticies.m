function update_verticies(vertex, goal, U, obs_set, start_v)
    for (i=1:vertex.num_pred)
        pred_v = vertex.pred_list(i);
        next_v = Vertex();
        if(~isempty(pred_v.x)) % Only process if predecessor vertex exists
            if(pred_v.x ~= goal.x &&...
               pred_v.y ~= goal.y)
                minimize_rhs(pred_v, obs_set); 
            end
            
            if(U.contains(pred_v))
                U.remove(pred_v);
            end
            
            if(pred_v.g ~= pred_v.rhs)
                U.push(pred_v, calc_key(pred_v, start_v));
            end
        end     
    end
end
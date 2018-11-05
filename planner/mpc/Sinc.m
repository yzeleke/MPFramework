function value = Sinc(theta)
    if theta == 0 
        value =1;
    else
        value = (sin(theta))/theta;
    end
end
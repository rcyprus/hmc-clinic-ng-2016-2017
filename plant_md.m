function y = plant_md(Ad, Bd, Cd, u, x)
    
    x_new = Ad*x + Bd*u;
    y = Cd*x_new;

end
function x = cross2D(a,b)
    % Finds magnitude of cross product of two 2D vectors
    
    % Add third dimension
    a = [a;0];
    b = [b;0];
    
    % Take cross product
    c = cross(a,b);
    
    % Get magnitude
    x = c(end);
end
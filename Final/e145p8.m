clear
M = [ 1 1 
    -1 -1];
u = [0];
x = [1];

for i = 1:10
    x(i+1) = x(i) + u(i);
    u(i+1) = -x(i) + -u(i);
    
end

z = [x; u]
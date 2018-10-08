e145hw1
A_c = [0 1
    -1  -.1];
B_c = [0 ;1];
C = [1 0];
D = 0;
system = ss(A_c,B_c,C,D);

dt = 0.01;
t = 0:dt:80;
u = cos(2*(t));
[y1, y2] = lsim(system,u,t)

hold on
plot(t,y1-.01)
legend('p1','p2')
hold off
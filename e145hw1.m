syms s
x = (s)/((s^2+4)*(s^2+0.1*s+1));
a = ilaplace(x);
fplot(a,[0,80])
hold on

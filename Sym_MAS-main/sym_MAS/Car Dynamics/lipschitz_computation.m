syms x


s = norm(x);
ds = diff(s);
ds = abs(ds);
x=1:10:100;
for i=1:10
    ds(i)=(abs(sign(x(i)))*abs(x(i)))/(abs(x(i))^2)^(1/2);
end
ds
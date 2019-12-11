function x = forward_sub(A,b)

n = length(b);
x = zeros(n,1);
x(1) = b(1) / A(1,1);
for i = 2:n
    x(i) = (b(i) - A(i,1:i-1)*x(1:i-1))./A(i,i);
end
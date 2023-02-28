function Q = decompose(a, n)

    Q = zeros(n+1);
    idx = triu(true(n+1));
    Q(idx') = a;
    Q = (Q + Q')./2;

end
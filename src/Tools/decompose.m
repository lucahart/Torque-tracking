function Q = decompose(a, n)
    % DECOMPOSE gets a polynomial with the coefficients a and order n and
    % decomposes it into the matrix form p(x) = a'v2 = v_1'Qv_1,
    % where v_m is the vector of monomials up until order m.
    % Inputs:
    %   a: Coefficients of polynomial p(x) = a'v_2.
    %   n: Polynomial dimension.
    % Outputs:
    %   Q: Matrix representation of polynomial p(x) = v_1'Qv_1.

    Q = zeros(n+1);
    idx = triu(true(n+1));
    Q(idx') = a;
    Q = (Q + Q')./2;

end
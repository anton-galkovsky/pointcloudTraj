#pragma once

class binomial_coefs {
public:
    binomial_coefs();

    int c(int n, int k);

private:
    const static int MAX_N = 13;

    int c_n_k[MAX_N][MAX_N];
};

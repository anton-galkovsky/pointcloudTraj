#include "pointcloudTraj/binomial_coefs.h"

int factorial_from(int n, int k) {
    int a = 1;
    for (int i = k; i <= n; i++) {
        a *= i;
    }
    return a;
}

binomial_coefs::binomial_coefs() {
    for (int n = 0; n < MAX_N; n++) {
        for (int k = 0; k < MAX_N; k++) {
            c_n_k[n][k] = factorial_from(n, k + 1) / factorial_from(n - k, 2);
        }
    }
}

int binomial_coefs::c(int n, int k) {
    return c_n_k[n][k];
}
#ifndef LINALG_H
#define LINALG_H

void transpose(int N, int M, double A[N][M], double At[M][N]);
void cofactor(int N, double A[N][N], double C[N][N]);
void add_mat(int N, int M, double A[N][M], double B[N][M], double C[N][M]);
void mult_mat(int NA, int L, int MB, double A[NA][L], double B[L][MB], double C[NA][MB]);
void mult_scal(int N, int M, double A[N][M], double x);
int inv(int N, double A[N][N], double B[N][N]);
double det(int N, double A[N][N]);
void printM(int NL, int NC, double M[NL][NC]);

#endif

#include <stdio.h>
#include <stdbool.h>

#include "linear_algebra.h"

// -----------MULTIPLICATION FUNCTION ----------
/**
 * @brief Generic matrix multiplication function  C = A*B
 * @param[in] NA 	nb of lines of A (and C)
 * @param[in] L 	nb of lines of B and nb of column of A
 * @param[in] MB	nb of column of B (and C)
 * @param[in] A		array of size NAxMA
 * @param[in] B 	array of size NBxMB
 * @param[out]C		array of size NAxMB, C = A*B
 * 
 */
void mult_mat(int NA, int L, int MB, double A[NA][L], double B[L][MB], double C[NA][MB])
{
	for(int i = 0; i<NA;i++)
		for(int j = 0; j<MB;j++)
		{
			C[i][j] = 0;
			for(int k = 0; k<L;k++){
				C[i][j] += A[i][k]*B[k][j];
			}
		}
}

/**
 * @brief Generic matrix multiplication function  C = A*B
 * @param[in] NA 	nb of lines of A (and C)
 * @param[in] L 	nb of lines of B and nb of column of A
 * @param[in] MB	nb of column of B (and C)
 * @param[in] A		array of size NAxMA
 * @param[in] B 	array of size NBxMB
 * @param[out]C		array of size NAxMB, C = A*B
 * 
 */
void mult_scal(int N, int M, double A[N][M], double x)
{
	for(int i = 0; i<N;i++)
		for(int j = 0; j<M;j++)
		{
			A[i][j] = A[i][j]*x;
		}
}

// -----------ADDITION FUNCTION ----------
/**
 * @brief Generic matrix addition function  C = A+B
 * @param[in] N 	nb of lines of A, B and C
 * @param[in] M		nb of column of A, B and C
 * @param[in] A		array of size NxM
 * @param[in] B 	array of size NxM
 * @param[out]C		array of size NxM, C = A+B
 * 
 */
void add_mat(int N, int M, double A[N][M], double B[N][M], double C[N][M])
{
	for(int i = 0; i<N;i++)
		for(int j = 0; j<M;j++)
				C[i][j] = A[i][j]+B[i][j];
}

// -----------INVERSION FUNCTION ----------
/**
 * @brief Generic matrix invers (B = inv(A)), use the Carley-Hamilton algorithm
 * @param[in] N 		nb of lines and columns of A and B
 * @param[in] A			array of size NxN
 * @param[out]B			invers of A
 * @return	  noninv	true if the matrix is not inversible
 * 
 */
bool inv(int N, double A[N][N], double B[N][N])
{
	double C[N][N], Ct[N][N];
	double d;
	double tol = 0.0000001;
	d = det(N,A);
	
	if (d < tol)
	{
		CATCH_ERR(d<tol,"matrix not inversible");
		return false;
	}
	cofactor(N,A,C);
	transpose(N,N,C,Ct);
	
	for(int i; i<N;i++)
		for(int j;j<N;j++)
			B[i][j] = Ct[i][j]/d;
	
	return true;
		
}

// -----------DET FUNCTION ----------
/**
 * @brief Generic matrix determinant, use the Carley-Hamilton algorithm
 * @param[in] N 	nb of lines and columns of A and B
 * @param[in] A		array of size NxN
 * @return 	  det	determinant of the matrix
 * 
 */
double det(int N, double A[N][N])
{
	float sign = 1;
	double d= 0;
	double pivot[N-1][N-1];
	if (N==2)
	{
		d = A[0][0]*A[1][1]-A[1][0]*A[0][1];
	}
	else
	{
		
		for(int i = 0; i<N;i++)
			{
				if(A[i][0])
				{
					
					for(int k = 0; k<N-1;k++)
						for(int l = 0; l<N-1;l++)
						{
							if(k<i)
							{
								pivot[k][l] = A[k][l+1];
								continue;
							}
							if(k>=i)
							{
								pivot[k][l] = A[k+1][l+1];
								continue;
							}
						}
					d += sign*A[i][0]*det(N-1,pivot);
					sign = -sign;
				}
			}
		
	}
	return d;
}

// -----------COFACTOR FUNCTION ----------
/**
 * @brief Generic comatrix calculation, use the Cramer rule
 * @param[in] N 	nb of lines and columns of A and B
 * @param[in] A		array of size NxN
 * @param[out]C 	Comatrix of A
 * 
 */
void cofactor(int N, double A[N][N], double C[N][N])
{
	double cofactor[N-1][N-1];
	double sign = 1;
	for(int i=0; i<N;i++)
	{
		for(int j=0; j<N;j++)
		{
			for(int k = 0; k<N-1;k++)
				for(int l = 0; l<N-1;l++)
				{
					if(k<i && l>=j)
					{
						cofactor[k][l] = A[k][l+1];
						continue;
					}
					if(k>=i && l>=j)
					{
						cofactor[k][l] = A[k+1][l+1];
						continue;
					}
					if(k<i && l<j)
					{
						cofactor[k][l] = A[k][l];
						continue;
					}
					if(k>=i && l<j)
					{
						cofactor[k][l] = A[k+1][l];
						continue;
					}
				}
			C[i][j] = sign*det(N-1,cofactor);
			sign = -sign;
		}
	}
}

// -----------TRANSPOSE FUNCTION ----------
/**
 * @brief Generic transpose calculation
 * @param[in] N 	nb of lines and columns of A and B
 * @param[in] A		array of size NxN
 * @param[out]At 	Comatrix of A
 * 
 */
void transpose(int N, int M, double A[N][M], double At[M][N])
{
	for(int i; i<N;i++)
		for(int j; j<M;j++)
			At[j][i] = A[i][j];
}

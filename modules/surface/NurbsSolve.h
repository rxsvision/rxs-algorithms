#pragma once

#include<Eigen/dense>
#include <iostream>
#include <cuda_runtime.h>
#include <cublas_v2.h>
#include <cusolverDn.h>
#include"czxTool.h"

class NurbsSolve
{
public:
	NurbsSolve();
	~NurbsSolve();
	static void init();
	static void finalize();
	void assign(unsigned rows, unsigned cols, unsigned dims);
	void K(unsigned i, unsigned j, double v)
	{
		m_Keig(i, j) = v;
	}
	void f(unsigned i, unsigned j, double v)
	{
		m_feig(i, j) = v;
	}
	double K(unsigned i, unsigned j)
	{
		return m_Keig(i, j);
	};

	bool solve();
	bool linearSolverQR2(Eigen::MatrixXd& m_Keig, Eigen::MatrixXd& m_xeig, Eigen::MatrixXd& m_feig);
	bool solveCuda();

	void x(unsigned i, unsigned j, double v)
	{
		m_xeig(i, j) = v;
	};

	double x(unsigned i, unsigned j)
	{
		return m_xeig(i, j);
	};

public:
	Eigen::MatrixXd 	m_feig;
	Eigen::MatrixXd 	m_Keig;
	//SparseMat 	m_Ksparse;
	bool 	m_quiet;
	Eigen::MatrixXd 	m_xeig;

	static cublasHandle_t cublasH;
	static cusolverDnHandle_t cusolverHandle;
	static int count;
	static mutex cuda_mtx;
};



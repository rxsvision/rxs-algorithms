#include "NurbsSolve.h"

#include <iostream>
#include <cuda_runtime.h>
#include <cublas_v2.h>
#include <cusolverDn.h>

int NurbsSolve::count = 0;
cublasHandle_t NurbsSolve::cublasH = NULL;
cusolverDnHandle_t NurbsSolve::cusolverHandle = NULL;
mutex NurbsSolve::cuda_mtx;

NurbsSolve::NurbsSolve()
{
    //if (count == 0)
    //{
    //    count++;
    //    cout << "new" << count <<endl;
    //    cublasCreate_v2(&cublasH);
    //    cusolverDnCreate(&cusolverHandle);
    //}
}

NurbsSolve::~NurbsSolve()
{
    //count--;
    //if (count == 0)
    //{
    //    cout << "delete" << endl;
    //    cusolverDnDestroy(cusolverHandle);
    //    cublasDestroy(cublasH);
    //}
}

void NurbsSolve::init()
{
    if (count == 0)
    {
        count++;
        cout << "new" << count << endl;
        cublasCreate(&cublasH);
        cusolverDnCreate(&cusolverHandle);
    }
}

void NurbsSolve::finalize()
{
    count--;
    if (count == 0)
    {
        cout << "delete" << endl;
        cusolverDnDestroy(cusolverHandle);
        cublasDestroy(cublasH);
    }
}


bool NurbsSolve::solve()
{
    //cout << "m_feig" << m_feig.size() << endl;
    //cout << "m_Keig" << m_Keig.cols() << endl;
    //cout << "m_xeig" << m_xeig.size() << endl;

    //if (m_Keig.size() > 1000000)
    //{
    //    //cout << m_Keig.size() << endl;
    //    //CzxTimer __(__func__);
    //    #pragma omp critical
    //    solveCuda();
    //    return true;
    //}
    //else
    //{
    //    m_xeig = m_Keig.householderQr().solve(m_feig);
    //    return true;
    //}

    {
        //CzxTimer _("CUDA的");
        return solveCuda();
    }

	//CzxTimer adgdfh("原始的");
	//m_xeig = m_Keig.colPivHouseholderQr().solve(m_feig 
    try
    {
        m_xeig = m_Keig.householderQr().solve(m_feig);
    }
    catch(bad_alloc e)
    {
        cout << "内存不足,无法更新nurbs" << endl;
    }
	//m_xeig = m_Keig.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(m_feig);

	return true;
}

bool NurbsSolve::linearSolverQR2(Eigen::MatrixXd& m_Keig, Eigen::MatrixXd& m_xeig, Eigen::MatrixXd& m_feig)
{
    #pragma omp critical
    {
        cusolverStatus_t status;
        // 获取矩阵数据的指针
        double* k_ptr = m_Keig.data();
        double* f_ptr = m_feig.data();


        double* cuda_k, * cuda_f;
        double* tau_device;
        double* cuda_Rx;
        double* workspace_device;
        int* devInfo;

        // 在CUDA中分配设备内存
        cudaMalloc((void**)&cuda_k, m_Keig.rows() * m_Keig.cols() * sizeof(double));
        cudaMalloc((void**)&cuda_f, m_feig.rows() * m_feig.cols() * sizeof(double));
        cudaMalloc((void**)&tau_device, m_Keig.cols() * sizeof(double));
        cudaMalloc((void**)&cuda_Rx, m_Keig.rows() * m_xeig.cols() * sizeof(double)); //

        // 将数据从主机内存复制到设备内存
        cudaMemcpy(cuda_k, k_ptr, m_Keig.rows() * m_Keig.cols() * sizeof(double), cudaMemcpyHostToDevice);
        cudaMemcpy(cuda_f, f_ptr, m_feig.rows() * m_feig.cols() * sizeof(double), cudaMemcpyHostToDevice);

        cudaMalloc((void**)&devInfo, sizeof(int));


        // 获取cusolverDnDgeqrf函数所需的工作空间大小
        int workspace_size;
        status = cusolverDnDgeqrf_bufferSize(cusolverHandle, m_Keig.rows(), m_Keig.cols(), cuda_k, m_Keig.rows(), &workspace_size);
        //checkCudaErrors(
        if (status != CUSOLVER_STATUS_SUCCESS) {
            std::cerr << "cusolverDnDgeqrf_bufferSize failed with error code: " << status << std::endl;
        }
        // 在设备上为 workspace_device 分配内存
        cudaMalloc(&workspace_device, workspace_size*sizeof(double));

        status = cusolverDnDgeqrf(cusolverHandle, m_Keig.rows(), m_Keig.cols(), cuda_k, m_Keig.rows(), tau_device, workspace_device, workspace_size, devInfo);

        if (status != CUSOLVER_STATUS_SUCCESS) {
            //int h_info;
            //cudaMemcpy(&h_info, devInfo, sizeof(int), cudaMemcpyDeviceToHost);
            std::cerr << "cusolverDnDgeqrf failed with error code: " << status << std::endl;
        }

        // 计算R*x=Q^T*F, C是R*x
        status = cusolverDnDormqr(cusolverHandle, CUBLAS_SIDE_LEFT, CUBLAS_OP_T, m_feig.rows(),
            m_feig.cols(), m_Keig.cols(), cuda_k, m_Keig.rows(), tau_device,
            cuda_f, m_feig.rows(), workspace_device, workspace_size, devInfo);
        if (status != CUSOLVER_STATUS_SUCCESS) {
            std::cerr << "cusolverDnDormqr failed with error code: " << status << std::endl;
        }


        // 解上三角方程 R * x = Q^T * b
        double alpha = 1.0;
        cublasStatus_t status_cublas = cublasDtrsm(cublasH, CUBLAS_SIDE_LEFT, CUBLAS_FILL_MODE_UPPER,
            CUBLAS_OP_N, CUBLAS_DIAG_NON_UNIT, m_xeig.rows(), m_xeig.cols(),
            &alpha, cuda_k, m_Keig.rows(), cuda_f, m_Keig.rows());
        if (status_cublas != CUBLAS_STATUS_SUCCESS) {
            fprintf(stderr, "cublasDtrsm failed with error code: %d\n", status_cublas);
        }


        Eigen::MatrixXd tmp(m_feig.rows(), m_feig.cols());
        cudaMemcpy(tmp.data(), cuda_f, sizeof(double) * m_feig.rows() * m_feig.cols(), cudaMemcpyDeviceToHost);

        m_xeig = tmp.block(0, 0, m_xeig.rows(), m_xeig.cols());
        //cout << "m_xeig" << endl;
        //cout << m_xeig << endl;


        // 释放设备内存
        cudaFree(cuda_k);
        cudaFree(cuda_f);
        cudaFree(cuda_Rx);
        cudaFree(tau_device);
        cudaFree(workspace_device);
        cudaFree(devInfo);
    }

    return true;
}


bool NurbsSolve::solveCuda()
{
    return linearSolverQR2(m_Keig, m_xeig, m_feig);
}

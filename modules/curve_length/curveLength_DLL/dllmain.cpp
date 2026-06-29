// dllmain.cpp : 定义 DLL 应用程序的入口点。
//#include "pch.h"
#include"BSPlineLength.h"


BOOL APIENTRY DllMain( HMODULE hModule,
                       DWORD  ul_reason_for_call,
                       LPVOID lpReserved
                     )
{
    switch (ul_reason_for_call)
    {
    case DLL_PROCESS_ATTACH:
    case DLL_THREAD_ATTACH:
    case DLL_THREAD_DETACH:
    case DLL_PROCESS_DETACH:
        break;
    }
    return TRUE;
}

__declspec(dllexport) vector<vector<vector<double>>> getLength(CP cloud, vector<CP>& fitted_x, vector<CP>& fitted_y)
{
    vector<vector<vector<double>>> ret;
    BSplineLength bsp;
    ret = bsp.process(cloud);
    fitted_x = bsp.fitted_x;
    fitted_y = bsp.fitted_y;
    return ret;
}
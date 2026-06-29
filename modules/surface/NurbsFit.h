#pragma once

#include"czxTool.h"
#include"NurbsSolve.h"
#include"NurbsSurface.h"
#include <Eigen/SparseLU>

#include <iostream>
#include <cublas_v2.h>

struct Parameter
{
	double interior_weight;
	double interior_smoothness;
	double interior_regularisation;

	double boundary_weight;
	double boundary_smoothness;
	double boundary_regularisation;

	unsigned regularisation_resU;
	unsigned regularisation_resV;

	Parameter(double intW = 1.0, double intS = 1e-6, double intR = 0.0, double bndW = 1.0,
		double bndS = 1e-6, double bndR = 0.0, unsigned regU = 0, unsigned regV = 0) :
		interior_weight(intW), interior_smoothness(intS), interior_regularisation(intR), boundary_weight(bndW),
		boundary_smoothness(bndS), boundary_regularisation(bndR), regularisation_resU(regU),
		regularisation_resV(regV)
	{
	}
};


class NurbsSolve;


class NurbsFit
{
public:
	NurbsFit(NurbsDataSurface* m_data, const NurbsSurface& ns);
	void refine(int dim);
	void assemble(Parameter param);
	void solve(double damp = 1.0);


	void init();
	void assembleBoundary(double wBnd, unsigned& row);
	void addPointConstraint(const Eigen::Vector2d& params, const Eigen::Vector3d& point, double weight, unsigned& row);
	int grc2gl(int I, int J);
	int lrc2gl(int E, int F, int i, int j);
	int gl2gr(int A)
		{
		    return (static_cast<int> (A / m_nurbs.CVCount(1)));
		} // global lexicographic in global row index
	int gl2gc(int A)
		{
		    return (static_cast<int> (A % m_nurbs.CVCount(1)));
		} // global lexicographic in global col index
	void assembleInterior(double wInt, unsigned& row);
	Vector2d findClosestElementMidPoint(NurbsSurface& nurbs, const Vector3d& pt);
	void addInteriorRegularisation(int order, int resU, int resV, double weight, unsigned& row);
	void addBoundaryRegularisation(int order, int resU, int resV, double weight, unsigned& row);
	void addCageInteriorRegularisation(double weight, unsigned& row);
	void addCageBoundaryRegularisation(double weight, int side, unsigned& row);
	void addCageCornerRegularisation(double weight, unsigned& row);
	void updateSurf(double damp);
	Vector2d inverseMapping(NurbsSurface& surface, const Vector3d& pt, const Vector2d& hint, double& error,
		Vector3d& p, Vector3d& tu, Vector3d& tv, int maxSteps, double accuracy, bool quiet = true);

	NurbsDataSurface* m_data;
	NurbsSurface m_nurbs;
private:

	
	double 	in_accuracy;
	int in_max_steps;
	std::vector< double > 	m_elementsU;
	std::vector< double > 	m_elementsV;
	double 	m_maxU;
	double 	m_maxV;
	double 	m_minU;
	double 	m_minV;
	bool 	m_quiet;
	NurbsSolve 	m_solver;
};


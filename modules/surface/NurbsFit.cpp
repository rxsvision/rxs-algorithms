#include "NurbsFit.h"

NurbsFit::NurbsFit(NurbsDataSurface* m_data, const NurbsSurface& ns)
{
	this->m_nurbs = ns;
	this->m_data = m_data;

	this->init();
}

void NurbsFit::refine(int dim)
{
	std::vector<double> xi;
	std::vector<double> elements = m_nurbs.getElementVector(dim);

	for (unsigned i = 0; i < elements.size() - 1; i++)
		xi.push_back(elements[i] + 0.5 * (elements[i + 1] - elements[i]));

	for (unsigned i = 0; i < xi.size(); i++)
		m_nurbs.InsertKnot(dim, xi[i], 1);

	m_elementsU = m_nurbs.getElementVector(0);
	m_elementsV = m_nurbs.getElementVector(1);
	m_minU = m_elementsU[0];
	m_minV = m_elementsV[0];
	m_maxU = m_elementsU[m_elementsU.size() - 1];
	m_maxV = m_elementsV[m_elementsV.size() - 1];
}

void NurbsFit::assemble(Parameter param)
{
	int nBnd = static_cast<int> (m_data->boundary.size());
	int nInt = static_cast<int> (m_data->interior.cols());
	int nCurInt = param.regularisation_resU * param.regularisation_resV;
	int nCurBnd = 2 * param.regularisation_resU + 2 * param.regularisation_resV;
	int nCageReg = (m_nurbs.m_cv_count[0] - 2) * (m_nurbs.m_cv_count[1] - 2);
	int nCageRegBnd = 2 * (m_nurbs.m_cv_count[0] - 1) + 2 * (m_nurbs.m_cv_count[1] - 1);

	if (param.boundary_weight <= 0.0)
		nBnd = 0;
	if (param.interior_weight <= 0.0)
		nInt = 0;
	if (param.boundary_regularisation <= 0.0)
		nCurBnd = 0;
	if (param.interior_regularisation <= 0.0)
		nCurInt = 0;
	if (param.interior_smoothness <= 0.0)
		nCageReg = 0;
	if (param.boundary_smoothness <= 0.0)
		nCageRegBnd = 0;

	int ncp = m_nurbs.m_cv_count[0] * m_nurbs.m_cv_count[1];
	int nrows = nBnd + nInt + nCurInt + nCurBnd + nCageReg + nCageRegBnd;

	m_solver.assign(nrows, ncp, 3);

	unsigned row = 0;

	// boundary points should lie on edges of surface
	if (nBnd > 0)
		assembleBoundary(param.boundary_weight, row);

	// interior points should lie on surface
	if (nInt > 0)
		assembleInterior(param.interior_weight, row);

	// minimal curvature on surface
	if (nCurInt > 0)
	{
		if (m_nurbs.Order(0) < 3 || m_nurbs.Order(1) < 3)
			printf("[FittingSurface::assemble] Error insufficient NURBS order to add curvature regularisation.\n");
		else
			addInteriorRegularisation(2, param.regularisation_resU, param.regularisation_resV,
				param.interior_regularisation / param.regularisation_resU, row);
	}

	// minimal curvature on boundary
	if (nCurBnd > 0)
	{
		if (m_nurbs.Order(0) < 3 || m_nurbs.Order(1) < 3)
			printf("[FittingSurface::assemble] Error insufficient NURBS order to add curvature regularisation.\n");
		else
			addBoundaryRegularisation(2, param.regularisation_resU, param.regularisation_resV,
				param.boundary_regularisation / param.regularisation_resU, row);
	}

	// cage regularisation
	if (nCageReg > 0)
		addCageInteriorRegularisation(param.interior_smoothness, row);

	if (nCageRegBnd > 0)
	{
		addCageBoundaryRegularisation(param.boundary_smoothness, pcl::on_nurbs::NORTH, row);
		addCageBoundaryRegularisation(param.boundary_smoothness, pcl::on_nurbs::SOUTH, row);
		addCageBoundaryRegularisation(param.boundary_smoothness, pcl::on_nurbs::WEST, row);
		addCageBoundaryRegularisation(param.boundary_smoothness, pcl::on_nurbs::EAST, row);
		addCageCornerRegularisation(param.boundary_smoothness * 2.0, row);
	}

}

void NurbsFit::solve(double damp)
{
	if (m_solver.solve())
		updateSurf(damp);
}

void NurbsFit::init()
{
	m_elementsU = m_nurbs.getElementVector(0);
	m_elementsV = m_nurbs.getElementVector(1);
	m_minU = m_elementsU[0];
	m_minV = m_elementsV[0];
	m_maxU = m_elementsU[m_elementsU.size() - 1];
	m_maxV = m_elementsV[m_elementsV.size() - 1];

	in_max_steps = 100;
	in_accuracy = 1e-4;

	m_quiet = true;
}

void NurbsFit::assembleBoundary(double wBnd, unsigned& row)
{
	m_data->boundary_line_start.clear();
	m_data->boundary_line_end.clear();
	m_data->boundary_error.clear();
	m_data->boundary_normals.clear();
	unsigned nBnd = static_cast<unsigned> (m_data->boundary.size());
	for (unsigned p = 0; p < nBnd; p++)
	{
		Vector3d& pcp = m_data->boundary[p];

		double error;
		Vector3d pt, tu, tv, n;
		Vector2d params = m_nurbs.inverseMappingBoundary(pcp, error, pt, tu, tv, in_max_steps, in_accuracy);
		m_data->boundary_error.push_back(error);

		if (p < m_data->boundary_param.size())
		{
			m_data->boundary_param[p] = params;
		}
		else
		{
			m_data->boundary_param.push_back(params);
		}

		n = tu.cross(tv);
		n.normalize();

		m_data->boundary_normals.push_back(n);
		m_data->boundary_line_start.push_back(pcp);
		m_data->boundary_line_end.push_back(pt);

		double w(wBnd);
		if (p < m_data->boundary_weight.size())
			w = m_data->boundary_weight[p];

		addPointConstraint(m_data->boundary_param[p], m_data->boundary[p], w, row);
	}
}


int NurbsFit::grc2gl(int I, int J)
{
	return m_nurbs.CVCount(1) * I + J;
} // global row/col index to global lexicographic index
int NurbsFit::lrc2gl(int E, int F, int i, int j)
{
	return grc2gl(E + i, F + j);
}

void NurbsFit::assembleInterior(double wInt, unsigned& row)
{
	m_data->interior_line_start.clear();
	m_data->interior_line_end.clear();
	m_data->interior_error.clear();
	m_data->interior_normals.clear();
	unsigned nInt = static_cast<unsigned> (m_data->interior.cols());
	for (unsigned p = 0; p < nInt; p++)
	{
		Vector3d pcp = m_data->interior.col(p);

		// inverse mapping
		Vector2d params;
		Vector3d pt, tu, tv, n;
		double error;
		if (p < m_data->interior_param.size())
		{
			params = m_nurbs.inverseMapping(pcp, m_data->interior_param[p], error, pt, tu, tv, in_max_steps, in_accuracy);
			m_data->interior_param[p] = params;
		}
		else
		{
			params = findClosestElementMidPoint(m_nurbs, pcp);
			params = m_nurbs.inverseMapping(pcp, params, error, pt, tu, tv, in_max_steps, in_accuracy);
			m_data->interior_param.push_back(params);
		}
		m_data->interior_error.push_back(error);

		n = tu.cross(tv);
		n.normalize();

		m_data->interior_normals.push_back(n);
		m_data->interior_line_start.push_back(pcp);
		m_data->interior_line_end.push_back(pt);

		double w(wInt);
		if (p < m_data->interior_weight.size())
			w = m_data->interior_weight[p];

		addPointConstraint(m_data->interior_param[p], m_data->interior.col(p), w, row);
	}
}

void NurbsFit::addInteriorRegularisation(int order, int resU, int resV, double weight, unsigned& row)
{
	double* N0 = new double[m_nurbs.Order(0) * m_nurbs.Order(0)];
	double* N1 = new double[m_nurbs.Order(1) * m_nurbs.Order(1)];

	double dU = (m_maxU - m_minU) / resU;
	double dV = (m_maxV - m_minV) / resV;

	for (int u = 0; u < resU; u++)
	{
		for (int v = 0; v < resV; v++)
		{

			Vector2d params;
			params(0) = m_minU + u * dU + 0.5 * dU;
			params(1) = m_minV + v * dV + 0.5 * dV;

			//                        printf("%f %f, %f %f\n", m_minU, dU, params(0), params(1));

			int E = ON_NurbsSpanIndex(m_nurbs.m_order[0], m_nurbs.m_cv_count[0], m_nurbs.m_knot[0], params(0), 0, 0);
			int F = ON_NurbsSpanIndex(m_nurbs.m_order[1], m_nurbs.m_cv_count[1], m_nurbs.m_knot[1], params(1), 0, 0);

			ON_EvaluateNurbsBasis(m_nurbs.Order(0), m_nurbs.m_knot[0] + E, params(0), N0);
			ON_EvaluateNurbsBasis(m_nurbs.Order(1), m_nurbs.m_knot[1] + F, params(1), N1);
			ON_EvaluateNurbsBasisDerivatives(m_nurbs.Order(0), m_nurbs.m_knot[0] + E, order, N0); // derivative order?
			ON_EvaluateNurbsBasisDerivatives(m_nurbs.Order(1), m_nurbs.m_knot[1] + F, order, N1);

			m_solver.f(row, 0, 0.0);
			m_solver.f(row, 1, 0.0);
			m_solver.f(row, 2, 0.0);

			for (int i = 0; i < m_nurbs.Order(0); i++)
			{

				for (int j = 0; j < m_nurbs.Order(1); j++)
				{

					m_solver.K(row, lrc2gl(E, F, i, j),
						weight * (N0[order * m_nurbs.Order(0) + i] * N1[j] + N0[i] * N1[order * m_nurbs.Order(1) + j]));

				} // i

			} // j

			row++;

		}
	}

	delete[] N1;
	delete[] N0;
}

void NurbsFit::addBoundaryRegularisation(int order, int resU, int resV, double weight, unsigned& row)
{
	double* N0 = new double[m_nurbs.Order(0) * m_nurbs.Order(0)];
	double* N1 = new double[m_nurbs.Order(1) * m_nurbs.Order(1)];

	double dU = (m_maxU - m_minU) / resU;
	double dV = (m_maxV - m_minV) / resV;

	for (int u = 0; u < resU; u++)
	{

		Vector2d params;
		params(0) = m_minU + u * dU + 0.5 * dU;
		params(1) = m_minV;

		int E = ON_NurbsSpanIndex(m_nurbs.m_order[0], m_nurbs.m_cv_count[0], m_nurbs.m_knot[0], params(0), 0, 0);
		int F = ON_NurbsSpanIndex(m_nurbs.m_order[1], m_nurbs.m_cv_count[1], m_nurbs.m_knot[1], params(1), 0, 0);

		ON_EvaluateNurbsBasis(m_nurbs.Order(0), m_nurbs.m_knot[0] + E, params(0), N0);
		ON_EvaluateNurbsBasis(m_nurbs.Order(1), m_nurbs.m_knot[1] + F, params(1), N1);
		ON_EvaluateNurbsBasisDerivatives(m_nurbs.Order(0), m_nurbs.m_knot[0] + E, order, N0); // derivative order?
		ON_EvaluateNurbsBasisDerivatives(m_nurbs.Order(1), m_nurbs.m_knot[1] + F, order, N1);

		m_solver.f(row, 0, 0.0);
		m_solver.f(row, 1, 0.0);
		m_solver.f(row, 2, 0.0);

		for (int i = 0; i < m_nurbs.Order(0); i++)
		{

			for (int j = 0; j < m_nurbs.Order(1); j++)
			{

				m_solver.K(row, lrc2gl(E, F, i, j),
					weight * (N0[order * m_nurbs.Order(0) + i] * N1[j] + N0[i] * N1[order * m_nurbs.Order(1) + j]));

			} // i

		} // j

		row++;

	}

	for (int u = 0; u < resU; u++)
	{

		Vector2d params;
		params(0) = m_minU + u * dU + 0.5 * dU;
		params(1) = m_maxV;

		int E = ON_NurbsSpanIndex(m_nurbs.m_order[0], m_nurbs.m_cv_count[0], m_nurbs.m_knot[0], params(0), 0, 0);
		int F = ON_NurbsSpanIndex(m_nurbs.m_order[1], m_nurbs.m_cv_count[1], m_nurbs.m_knot[1], params(1), 0, 0);

		ON_EvaluateNurbsBasis(m_nurbs.Order(0), m_nurbs.m_knot[0] + E, params(0), N0);
		ON_EvaluateNurbsBasis(m_nurbs.Order(1), m_nurbs.m_knot[1] + F, params(1), N1);
		ON_EvaluateNurbsBasisDerivatives(m_nurbs.Order(0), m_nurbs.m_knot[0] + E, order, N0); // derivative order?
		ON_EvaluateNurbsBasisDerivatives(m_nurbs.Order(1), m_nurbs.m_knot[1] + F, order, N1);

		m_solver.f(row, 0, 0.0);
		m_solver.f(row, 1, 0.0);
		m_solver.f(row, 2, 0.0);

		for (int i = 0; i < m_nurbs.Order(0); i++)
		{

			for (int j = 0; j < m_nurbs.Order(1); j++)
			{

				m_solver.K(row, lrc2gl(E, F, i, j),
					weight * (N0[order * m_nurbs.Order(0) + i] * N1[j] + N0[i] * N1[order * m_nurbs.Order(1) + j]));

			} // i

		} // j

		row++;

	}

	for (int v = 0; v < resV; v++)
	{

		Vector2d params;
		params(0) = m_minU;
		params(1) = m_minV + v * dV + 0.5 * dV;

		int E = ON_NurbsSpanIndex(m_nurbs.m_order[0], m_nurbs.m_cv_count[0], m_nurbs.m_knot[0], params(0), 0, 0);
		int F = ON_NurbsSpanIndex(m_nurbs.m_order[1], m_nurbs.m_cv_count[1], m_nurbs.m_knot[1], params(1), 0, 0);

		ON_EvaluateNurbsBasis(m_nurbs.Order(0), m_nurbs.m_knot[0] + E, params(0), N0);
		ON_EvaluateNurbsBasis(m_nurbs.Order(1), m_nurbs.m_knot[1] + F, params(1), N1);
		ON_EvaluateNurbsBasisDerivatives(m_nurbs.Order(0), m_nurbs.m_knot[0] + E, order, N0); // derivative order?
		ON_EvaluateNurbsBasisDerivatives(m_nurbs.Order(1), m_nurbs.m_knot[1] + F, order, N1);

		m_solver.f(row, 0, 0.0);
		m_solver.f(row, 1, 0.0);
		m_solver.f(row, 2, 0.0);

		for (int i = 0; i < m_nurbs.Order(0); i++)
		{

			for (int j = 0; j < m_nurbs.Order(1); j++)
			{

				m_solver.K(row, lrc2gl(E, F, i, j),
					weight * (N0[order * m_nurbs.Order(0) + i] * N1[j] + N0[i] * N1[order * m_nurbs.Order(1) + j]));

			} // i

		} // j

		row++;

	}

	for (int v = 0; v < resV; v++)
	{

		Vector2d params;
		params(0) = m_maxU;
		params(1) = m_minV + v * dV + 0.5 * dV;

		int E = ON_NurbsSpanIndex(m_nurbs.m_order[0], m_nurbs.m_cv_count[0], m_nurbs.m_knot[0], params(0), 0, 0);
		int F = ON_NurbsSpanIndex(m_nurbs.m_order[1], m_nurbs.m_cv_count[1], m_nurbs.m_knot[1], params(1), 0, 0);

		ON_EvaluateNurbsBasis(m_nurbs.Order(0), m_nurbs.m_knot[0] + E, params(0), N0);
		ON_EvaluateNurbsBasis(m_nurbs.Order(1), m_nurbs.m_knot[1] + F, params(1), N1);
		ON_EvaluateNurbsBasisDerivatives(m_nurbs.Order(0), m_nurbs.m_knot[0] + E, order, N0); // derivative order?
		ON_EvaluateNurbsBasisDerivatives(m_nurbs.Order(1), m_nurbs.m_knot[1] + F, order, N1);

		m_solver.f(row, 0, 0.0);
		m_solver.f(row, 1, 0.0);
		m_solver.f(row, 2, 0.0);

		for (int i = 0; i < m_nurbs.Order(0); i++)
		{

			for (int j = 0; j < m_nurbs.Order(1); j++)
			{

				m_solver.K(row, lrc2gl(E, F, i, j),
					weight * (N0[order * m_nurbs.Order(0) + i] * N1[j] + N0[i] * N1[order * m_nurbs.Order(1) + j]));

			} // i

		} // j

		row++;

	}

	delete[] N1;
	delete[] N0;
}

void NurbsFit::addCageInteriorRegularisation(double weight, unsigned& row)
{
	for (int i = 1; i < (m_nurbs.m_cv_count[0] - 1); i++)
	{
		for (int j = 1; j < (m_nurbs.m_cv_count[1] - 1); j++)
		{

			m_solver.f(row, 0, 0.0);
			m_solver.f(row, 1, 0.0);
			m_solver.f(row, 2, 0.0);

			m_solver.K(row, grc2gl(i + 0, j + 0), -4.0 * weight);
			m_solver.K(row, grc2gl(i + 0, j - 1), 1.0 * weight);
			m_solver.K(row, grc2gl(i + 0, j + 1), 1.0 * weight);
			m_solver.K(row, grc2gl(i - 1, j + 0), 1.0 * weight);
			m_solver.K(row, grc2gl(i + 1, j + 0), 1.0 * weight);

			row++;
		}
	}
}

void NurbsFit::addCageBoundaryRegularisation(double weight, int side, unsigned& row)
{
	int i = 0;
	int j = 0;

	switch (side)
	{
	case pcl::on_nurbs::SOUTH:
		j = m_nurbs.m_cv_count[1] - 1;
	case pcl::on_nurbs::NORTH:
		for (i = 1; i < (m_nurbs.m_cv_count[0] - 1); i++)
		{

			m_solver.f(row, 0, 0.0);
			m_solver.f(row, 1, 0.0);
			m_solver.f(row, 2, 0.0);

			m_solver.K(row, grc2gl(i + 0, j), -2.0 * weight);
			m_solver.K(row, grc2gl(i - 1, j), 1.0 * weight);
			m_solver.K(row, grc2gl(i + 1, j), 1.0 * weight);

			row++;
		}
		break;

	case pcl::on_nurbs::EAST:
		i = m_nurbs.m_cv_count[0] - 1;
	case pcl::on_nurbs::WEST:
		for (j = 1; j < (m_nurbs.m_cv_count[1] - 1); j++)
		{

			m_solver.f(row, 0, 0.0);
			m_solver.f(row, 1, 0.0);
			m_solver.f(row, 2, 0.0);

			m_solver.K(row, grc2gl(i, j + 0), -2.0 * weight);
			m_solver.K(row, grc2gl(i, j - 1), 1.0 * weight);
			m_solver.K(row, grc2gl(i, j + 1), 1.0 * weight);

			row++;
		}
		break;
	}
}

void NurbsFit::addCageCornerRegularisation(double weight, unsigned& row)
{
	{ // NORTH-WEST
		int i = 0;
		int j = 0;

		m_solver.f(row, 0, 0.0);
		m_solver.f(row, 1, 0.0);
		m_solver.f(row, 2, 0.0);

		m_solver.K(row, grc2gl(i + 0, j + 0), -2.0 * weight);
		m_solver.K(row, grc2gl(i + 1, j + 0), 1.0 * weight);
		m_solver.K(row, grc2gl(i + 0, j + 1), 1.0 * weight);

		row++;
	}

	{ // NORTH-EAST
		int i = m_nurbs.m_cv_count[0] - 1;
		int j = 0;

		m_solver.f(row, 0, 0.0);
		m_solver.f(row, 1, 0.0);
		m_solver.f(row, 2, 0.0);

		m_solver.K(row, grc2gl(i + 0, j + 0), -2.0 * weight);
		m_solver.K(row, grc2gl(i - 1, j + 0), 1.0 * weight);
		m_solver.K(row, grc2gl(i + 0, j + 1), 1.0 * weight);

		row++;
	}

	{ // SOUTH-EAST
		int i = m_nurbs.m_cv_count[0] - 1;
		int j = m_nurbs.m_cv_count[1] - 1;

		m_solver.f(row, 0, 0.0);
		m_solver.f(row, 1, 0.0);
		m_solver.f(row, 2, 0.0);

		m_solver.K(row, grc2gl(i + 0, j + 0), -2.0 * weight);
		m_solver.K(row, grc2gl(i - 1, j + 0), 1.0 * weight);
		m_solver.K(row, grc2gl(i + 0, j - 1), 1.0 * weight);

		row++;
	}

	{ // SOUTH-WEST
		int i = 0;
		int j = m_nurbs.m_cv_count[1] - 1;

		m_solver.f(row, 0, 0.0);
		m_solver.f(row, 1, 0.0);
		m_solver.f(row, 2, 0.0);

		m_solver.K(row, grc2gl(i + 0, j + 0), -2.0 * weight);
		m_solver.K(row, grc2gl(i + 1, j + 0), 1.0 * weight);
		m_solver.K(row, grc2gl(i + 0, j - 1), 1.0 * weight);

		row++;
	}

}

void NurbsFit::updateSurf(double damp)
{
	int ncp = m_nurbs.m_cv_count[0] * m_nurbs.m_cv_count[1];

	for (int A = 0; A < ncp; A++)
	{

		int I = gl2gr(A);
		int J = gl2gc(A);

		PointT cp_prev;
		m_nurbs.GetCV(I, J, cp_prev);

		PointT cp;
		cp.x = cp_prev.x + damp * (m_solver.x(A, 0) - cp_prev.x);
		cp.y = cp_prev.y + damp * (m_solver.x(A, 1) - cp_prev.y);
		cp.z = cp_prev.z + damp * (m_solver.x(A, 2) - cp_prev.z);

		m_nurbs.SetCV(I, J, cp);

	}

}

Vector2d NurbsFit::inverseMapping(NurbsSurface& surface, const Vector3d& pt, const Vector2d& hint, double& error,
	Vector3d& p, Vector3d& tu, Vector3d& tv, int maxSteps, double accuracy, bool quiet) 
{
	return surface.inverseMapping(pt, hint, error, p, tu, tv, maxSteps, accuracy, quiet);
}



Vector2d NurbsFit::findClosestElementMidPoint(NurbsSurface& nurbs, const Vector3d& pt)
{
	Vector2d hint;
	Vector3d r;
	std::vector<double> elementsU = nurbs.getElementVector(0);
	std::vector<double> elementsV = nurbs.getElementVector(1);

	double d_shortest(DBL_MAX);
	for (unsigned i = 0; i < elementsU.size() - 1; i++)
	{
		for (unsigned j = 0; j < elementsV.size() - 1; j++)
		{
			double points[3];
			double d;

			double xi = elementsU[i] + 0.5 * (elementsU[i + 1] - elementsU[i]);
			double eta = elementsV[j] + 0.5 * (elementsV[j + 1] - elementsV[j]);

			nurbs.Evaluate(xi, eta, 0, 3, points);
			r(0) = points[0] - pt(0);
			r(1) = points[1] - pt(1);
			r(2) = points[2] - pt(2);

			d = r.squaredNorm();

			if ((i == 0 && j == 0) || d < d_shortest)
			{
				d_shortest = d;
				hint(0) = xi;
				hint(1) = eta;
			}
		}
	}

	return hint;
}

void NurbsFit::addPointConstraint(const Eigen::Vector2d& params, const Eigen::Vector3d& point, double weight, unsigned& row)
{
	double* N0 = new double[m_nurbs.Order(0) * m_nurbs.Order(0)];
	double* N1 = new double[m_nurbs.Order(1) * m_nurbs.Order(1)];

	int E = ON_NurbsSpanIndex(m_nurbs.m_order[0], m_nurbs.m_cv_count[0], m_nurbs.m_knot[0], params(0), 0, 0);
	int F = ON_NurbsSpanIndex(m_nurbs.m_order[1], m_nurbs.m_cv_count[1], m_nurbs.m_knot[1], params(1), 0, 0);

	ON_EvaluateNurbsBasis(m_nurbs.Order(0), m_nurbs.m_knot[0] + E, params(0), N0);
	ON_EvaluateNurbsBasis(m_nurbs.Order(1), m_nurbs.m_knot[1] + F, params(1), N1);

	m_solver.f(row, 0, point(0) * weight);
	m_solver.f(row, 1, point(1) * weight);
	m_solver.f(row, 2, point(2) * weight);

	for (int i = 0; i < m_nurbs.Order(0); i++)
	{

		for (int j = 0; j < m_nurbs.Order(1); j++)
		{

			m_solver.K(row, lrc2gl(E, F, i, j), weight * N0[i] * N1[j]);

		} // j

	} // i

	row++;

	delete[] N1;
	delete[] N0;
}

void NurbsSolve::assign(unsigned rows, unsigned cols, unsigned dims)
{
	m_Keig = Eigen::MatrixXd::Zero(rows, cols);
	m_xeig = Eigen::MatrixXd::Zero(cols, dims);
	m_feig = Eigen::MatrixXd::Zero(rows, dims);
}
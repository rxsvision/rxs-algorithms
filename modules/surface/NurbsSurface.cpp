#include "NurbsSurface.h"
#include "NurbsCommon.hpp"


//这些数值需要重新换
//#define ON_RELATIVE_TOLERANCE   2.    5625e-13
//
//#define ON_ZERO_TOLERANCE   2.   25e-10
//#define ON_SQRT_EPSILON   1. 850e-8
//
//#define ON_UNSET_VALUE   -1. 1321e+308

#define 	ON_IS_FINITE(x)   (_finite(x)?true:false)
#define ON_IsValid(x)  (x != ON_UNSET_VALUE && ON_IS_FINITE(x))


namespace czx
{
	int ON_SearchMonotoneArray(const double* array, int length, double t);
	bool ON_PointsAreCoincident(
		int dim,
		int is_rat,
		const double* pointA,
		const double* pointB
	);
	int ON_NurbsSpanIndex(
		int order,          // (>=2)
		int cv_count,
		const double* knot, // knot[] array or length ON_KnotCount(order,cv_count)
		double t,           // evaluation parameter
		int side,           // side 0 = default, -1 = from below, +1 = from above
		int hint            // hint (or 0 if no hint available)
	);
	bool ON_IsPointGridClosed(
		int dim,
		bool is_rat,
		int point_count0, int point_count1,
		int point_stride0, int point_stride1,
		const double* p,
		int dir
	);
	bool ON_IsKnotVectorClamped(
		int order,
		int cv_count,
		const double* knot,
		int end = 2 // (default = 2) 0 = left end, 1 = right end, 2 = both
	);
	bool ON_IsKnotVectorPeriodic(
		int order,
		int cv_count,
		const double* knot
	);
	bool ON_EvaluateNurbsSurfaceSpan(
		int dim,
		int is_rat,
		int order0, int order1,
		const double* knot0,
		const double* knot1,
		int cv_stride0, int cv_stride1,
		const double* cv0, // cv at "lower left" of bispan
		int der_count,
		double t0, double t1,
		int v_stride,
		double* v      // returns values
	);
	bool ON_EvaluateNurbsBasis(int order, const double* knot,
		double t, double* N);
	bool ON_EvaluateQuotientRule2(int dim, int der_count, int v_stride, double* v);
	bool ON_EvaluateNurbsBasisDerivatives(int order, const double* knot,
		int der_count, double* N);

	bool ON_GetKnotVectorDomain(
		int order,
		int cv_count,
		const double* knot,
		double* k0, double* k1
	)
	{
		if (order < 2 || cv_count < order || knot == NULL)
			return false;
		if (k0)
			*k0 = knot[order - 2];
		if (k1)
			*k1 = knot[cv_count - 1];
		return true;
	}

	int ON_ComparePoint( // returns 
								  // -1: first < second
								  //  0: first == second
								  // +1: first > second
		int dim,
		bool is_rat,
		const double* pointA,
		const double* pointB
	);
}


void pca(const Eigen::MatrixXd data, Eigen::Vector3d& mean, Eigen::Matrix3d& eigenvectors,
	Eigen::Vector3d& eigenvalues);

static void ConvertFromCurve(ON_NurbsCurve& crv, int dir, NurbsSurface& srf);
static void ConvertToCurve(NurbsSurface& srf, int dir, ON_NurbsCurve& crv);


void
NurbsSurface::initNurbsPCA(int order, NurbsDataSurface* m_data, Eigen::Vector3d z)
{
	//Initialize();

	Eigen::Vector3d mean;
	Eigen::Matrix3d eigenvectors;
	Eigen::Vector3d eigenvalues;

	unsigned s = m_data->interior.cols();

	pca(m_data->interior, mean, eigenvectors, eigenvalues);

	bool flip(false);
	if (eigenvectors.col(2).dot(z) < 0.0)
		flip = true;

	eigenvalues = eigenvalues / s; // seems that the eigenvalues are dependent on the number of points (???)


	Eigen::Vector3d sigma(sqrt(eigenvalues(0)), sqrt(eigenvalues(1)), sqrt(eigenvalues(2)));

	//ON_NurbsSurface nurbs(3, false, order, order, order, order);
	Create(3, false, order, order, order, order);

	MakeClampedUniformKnotVector(0, 1.0);
	MakeClampedUniformKnotVector(1, 1.0);
	// +- 2 sigma -> 95,45 % aller Messwerte

	double dcu = (4.0 * sigma(0)) / (Order(0) - 1);
	double dcv = (4.0 * sigma(1)) / (Order(1) - 1);

	Eigen::Vector3d cv_t, cv;
	for (int i = 0; i < Order(0); i++)
	{
		for (int j = 0; j < Order(1); j++)
		{
			cv(0) = -2.0 * sigma(0) + dcu * i;
			cv(1) = -2.0 * sigma(1) + dcv * j;
			cv(2) = 0.0;
			cv_t = eigenvectors * cv + mean;
			if (flip)
				SetCV(Order(0) - 1 - i, j, PointT(cv_t(0), cv_t(1), cv_t(2)));
			else
				SetCV(i, j, PointT(cv_t(0), cv_t(1), cv_t(2)));
		}
	}
}

NurbsSurface::NurbsSurface()
{
	m_dim = 0;
	m_is_rat = 0;
	m_order[0] = 0;
	m_order[1] = 0;
	m_cv_count[0] = 0;
	m_cv_count[1] = 0;
	m_knot_capacity[0] = 0;
	m_knot_capacity[1] = 0;
	m_knot[0] = 0;
	m_knot[1] = 0;
	m_cv_stride[0] = 0;
	m_cv_stride[1] = 0;
	m_cv_capacity = 1;
	m_cv = new double[1];
}

// 拷贝构造函数的实现
NurbsSurface::NurbsSurface(const NurbsSurface& other) {
	// 深度拷贝指针成员
	m_cv_capacity = other.m_cv_capacity;
	m_cv_count[0] = other.m_cv_count[0];
	m_cv_count[1] = other.m_cv_count[1];
	m_cv_stride[0] = other.m_cv_stride[0];
	m_cv_stride[1] = other.m_cv_stride[1];
	m_knot_capacity[0] = other.m_knot_capacity[0];
	m_knot_capacity[1] = other.m_knot_capacity[1];

	m_dim = other.m_dim;
	m_is_rat = other.m_is_rat;
	m_order[0] = other.m_order[0];
	m_order[1] = other.m_order[1];

	// 分配新的内存并复制数据
	m_cv = new double[m_cv_capacity];
	std::copy(other.m_cv, other.m_cv + m_cv_capacity, m_cv);

	// 为 m_knot 分配新的内存并复制数据
	for (int i = 0; i < 2; ++i) {
		m_knot[i] = new double[m_knot_capacity[i]];
		std::copy(other.m_knot[i], other.m_knot[i] + m_knot_capacity[i], m_knot[i]);
	}
}

NurbsSurface& NurbsSurface::operator=(const NurbsSurface& other) {
	// 深度拷贝指针成员
	m_cv_capacity = other.m_cv_capacity;
	m_cv_count[0] = other.m_cv_count[0];
	m_cv_count[1] = other.m_cv_count[1];
	m_cv_stride[0] = other.m_cv_stride[0];
	m_cv_stride[1] = other.m_cv_stride[1];
	m_knot_capacity[0] = other.m_knot_capacity[0];
	m_knot_capacity[1] = other.m_knot_capacity[1];
	m_dim = other.m_dim;
	m_is_rat = other.m_is_rat;
	m_order[0] = other.m_order[0];
	m_order[1] = other.m_order[1];

	// 分配新的内存并复制数据
	m_cv = new double[m_cv_capacity];
	std::copy(other.m_cv, other.m_cv + m_cv_capacity, m_cv);

	// 为 m_knot 分配新的内存并复制数据
	for (int i = 0; i < 2; ++i) {
		m_knot[i] = new double[m_knot_capacity[i]];
		std::copy(other.m_knot[i], other.m_knot[i] + m_knot_capacity[i], m_knot[i]);
	}
	return *this;
}


bool NurbsSurface::Create(
	int dim,      // dimension (>= 1)
	bool is_rat,  // true to make a rational NURBS
	int order0,    // order (>= 2)
	int order1,    // order (>= 2)
	int cv_count0,  // cv count0 (>= order0) 
	int cv_count1   // cv count1 (>= order1)
)
{
	//DestroySurfaceTree();
	if (dim < 1)
		return false;
	if (order0 < 2)
		return false;
	if (order1 < 2)
		return false;
	if (cv_count0 < order0)
		return false;
	if (cv_count1 < order1)
		return false;
	m_dim = dim;
	m_is_rat = (is_rat) ? true : false;
	m_order[0] = order0;
	m_order[1] = order1;
	m_cv_count[0] = cv_count0;
	m_cv_count[1] = cv_count1;
	m_cv_stride[1] = (m_is_rat) ? m_dim + 1 : m_dim;
	m_cv_stride[0] = m_cv_stride[1] * m_cv_count[1];
	bool rc = ReserveKnotCapacity(0, KnotCount(0));
	if (!ReserveKnotCapacity(1, KnotCount(1)))
		rc = false;
	if (!ReserveCVCapacity(m_cv_count[0] * m_cv_count[1] * m_cv_stride[1]))
		rc = false;
	return rc;
}

int NurbsSurface::KnotCount(int dir)
{
	return ON_KnotCount(m_order[dir ? 1 : 0], m_cv_count[dir ? 1 : 0]);
}

bool NurbsSurface::ReserveKnotCapacity(int dir, int capacity)
{
	if (dir)
		dir = 1;
	if (m_knot_capacity[dir] < capacity) {
		if (m_knot[dir]) {
			if (m_knot_capacity[dir]) {
				m_knot[dir] = (double*)realloc(m_knot[dir], capacity * sizeof(*m_knot[dir]));
				m_knot_capacity[dir] = (m_knot[dir]) ? capacity : 0;

			}
			// else user supplied m_knot[] array

		}
		else {
			m_knot[dir] = (double*)malloc(capacity * sizeof(*m_knot[dir]));
			m_knot_capacity[dir] = (m_knot[dir]) ? capacity : 0;

		}

	}
	return (m_knot[dir]) ? true : false;
}

bool NurbsSurface::ReserveCVCapacity(int capacity)
{
	if (m_cv_capacity < capacity) {
		if (m_cv) {
			if (m_cv_capacity) {
				m_cv = (double*)realloc(m_cv, capacity * sizeof(*m_cv));
				m_cv_capacity = (m_cv) ? capacity : 0;

			}
			// else user supplied m_cv[] array

		}
		else {
			m_cv = (double*)malloc(capacity * sizeof(*m_cv));
			m_cv_capacity = (m_cv) ? capacity : 0;

		}

	}
	return (m_cv) ? true : false;
}


bool NurbsSurface::MakeClampedUniformKnotVector(
	int dir,
	double delta
)
{
	if (dir < 0 || dir > 1)
		return false;
	ReserveKnotCapacity(dir, ON_KnotCount(m_order[dir], m_cv_count[dir]));
	return ON_MakeClampedUniformKnotVector(m_order[dir], m_cv_count[dir], m_knot[dir], delta);
}

bool NurbsSurface::ON_MakeClampedUniformKnotVector(
	int order,
	int cv_count,
	double* knot,
	double delta
)
{
	bool rc = false;
	if (order >= 2 && cv_count >= order && knot != NULL && delta > 0.0)
	{
		double k;
		int i;
		for (i = order - 2, k = 0.0; i < cv_count; i++, k += delta)
			knot[i] = k;
		ON_ClampKnotVector(order, cv_count, knot, 2);
		rc = true;
	}
	return rc;
}

bool NurbsSurface::ON_ClampKnotVector(
	int order,          // order (>=2)
	int cv_count,       // cv count
	double* knot,       // knot[] array
	int end             // 0 = clamp start, 1 = clamp end, 2 = clamp both ends
)
{
	// sets initial/final order-2 knot values to match knot[order-2]/knot[cv_count-1]
	bool rc = false;
	int i, i0;
	if (knot && order >= 2 && cv_count >= order) {
		if (end == 0 || end == 2) {
			i0 = order - 2;
			for (i = 0; i < i0; i++) {
				knot[i] = knot[i0];

			}
			rc = true;

		}
		if (end == 1 || end == 2) {
			const int knot_count = ON_KnotCount(order, cv_count);
			i0 = cv_count - 1;
			for (i = i0 + 1; i < knot_count; i++) {
				knot[i] = knot[i0];

			}
			rc = true;

		}

	}
	return rc;
}

Vector2d NurbsSurface::inverseMapping(const Vector3d& pt, const Vector2d& hint, double& error,
	Vector3d& p, Vector3d& tu, Vector3d& tv, int maxSteps, double accuracy, bool quiet) 
{

	double pointAndTangents[9];

	Vector2d current, delta;
	Matrix2d A;
	Vector2d b;
	Vector3d r;
	//std::vector<double> elementsU = getElementVector(0);
	//std::vector<double> elementsV = getElementVector(1);
	//double minU = elementsU[0];
	//double minV = elementsV[0];
	//double maxU = elementsU[elementsU.size() - 1];
	//double maxV = elementsV[elementsV.size() - 1];

	//czx 让落在边界之外的点也能被拟合到
	double minU = -0.5;
	double minV = -0.5;
	double maxU = 1.5;
	double maxV = 1.5;

	current = hint;

	for (int k = 0; k < maxSteps; k++)
	{

		Evaluate(current(0), current(1), 1, 3, pointAndTangents);
		p(0) = pointAndTangents[0];
		p(1) = pointAndTangents[1];
		p(2) = pointAndTangents[2];
		tu(0) = pointAndTangents[3];
		tu(1) = pointAndTangents[4];
		tu(2) = pointAndTangents[5];
		tv(0) = pointAndTangents[6];
		tv(1) = pointAndTangents[7];
		tv(2) = pointAndTangents[8];

		r = p - pt;

		b(0) = -r.dot(tu);
		b(1) = -r.dot(tv);

		A(0, 0) = tu.dot(tu);
		A(0, 1) = tu.dot(tv);
		A(1, 0) = A(0, 1);
		A(1, 1) = tv.dot(tv);

		delta = A.ldlt().solve(b);

		if (delta.norm() < accuracy)
		{

			error = r.norm();
			return current;

		}
		else
		{
			current = current + delta;

			if (current(0) < minU)
				current(0) = minU;
			else if (current(0) > maxU)
				current(0) = maxU;

			if (current(1) < minV)
				current(1) = minV;
			else if (current(1) > maxV)
				current(1) = maxV;

		}

	}

	error = r.norm();

	if (!quiet)
	{
		printf("[FittingSurface::inverseMapping] Warning: Method did not converge (%e %d)\n", accuracy, maxSteps);
		printf("  %f %f ... %f %f\n", hint(0), hint(1), current(0), current(1));
	}

	return current;

}

bool NurbsSurface::Evaluate(double s, double t, int der_count, int v_stride, double* v, int side, int hint[2]) const
{
	bool rc = false;
	int span_index[2];
	span_index[0] = ON_NurbsSpanIndex(m_order[0], m_cv_count[0], m_knot[0], s, (side == 2 || side == 3) ? -1 : 1, (hint) ? hint[0] : 0);
	span_index[1] = ON_NurbsSpanIndex(m_order[1], m_cv_count[1], m_knot[1], t, (side == 3 || side == 4) ? -1 : 1, (hint) ? hint[1] : 0);
	rc = ON_EvaluateNurbsSurfaceSpan(
		m_dim, m_is_rat,
		m_order[0], m_order[1],
		m_knot[0] + span_index[0],
		m_knot[1] + span_index[1],
		m_cv_stride[0], m_cv_stride[1],
		m_cv + (span_index[0] * m_cv_stride[0] + span_index[1] * m_cv_stride[1]),
		der_count,
		s, t,
		v_stride, v
	);
	if (hint) {
		hint[0] = span_index[0];
		hint[1] = span_index[1];

	}
	return rc;
}

std::vector<double> NurbsSurface::getElementVector(int dim)
{
	std::vector<double> result;

	int idx_min = 0;
	int idx_max = KnotCount(dim) - 1;
	if (IsClosed(dim))
	{
		idx_min = Order(dim) - 2;
		idx_max = KnotCount(dim) - Order(dim) + 1;
	}

	const double* knots = Knot(dim);

	result.push_back(knots[idx_min]);

	//for(int E=(m_nurbs.Order(0)-2); E<(m_nurbs.KnotCount(0)-m_nurbs.Order(0)+2); E++) {
	for (int E = idx_min + 1; E <= idx_max; E++)
	{

		if (knots[E] != knots[E - 1]) // do not count double knots
			result.push_back(knots[E]);

	}

	return result;
}

bool NurbsSurface::IsClosed(int dir) const
{
	bool bIsClosed = false;
	if (dir >= 0 && dir <= 1 && m_dim > 0)
	{
		if (czx::ON_IsKnotVectorClamped(m_order[dir], m_cv_count[dir], m_knot[dir]))
		{
			const double* corners[4];
			corners[0] = CV(0, 0);
			corners[(0 == dir) ? 1 : 2] = CV(m_cv_count[0] - 1, 0);
			corners[(0 == dir) ? 2 : 1] = CV(0, m_cv_count[1] - 1);
			corners[3] = CV(m_cv_count[0] - 1, m_cv_count[1] - 1);
			if (ON_PointsAreCoincident(m_dim, m_is_rat, corners[0], corners[1])
				&& ON_PointsAreCoincident(m_dim, m_is_rat, corners[2], corners[3])
				&& czx::ON_IsPointGridClosed(m_dim, m_is_rat, m_cv_count[0], m_cv_count[1], m_cv_stride[0], m_cv_stride[1], m_cv, dir)
				)
			{
				bIsClosed = true;
			}
		}
		else if (IsPeriodic(dir))
		{
			bIsClosed = true;
		}
	}
	return bIsClosed;
}

bool NurbsSurface::IsPeriodic(int dir) const
{
	bool bIsPeriodic = false;
	if (dir >= 0 && dir <= 1)
	{
		int k;
		bIsPeriodic = ON_IsKnotVectorPeriodic(m_order[dir], m_cv_count[dir], m_knot[dir]);
		if (bIsPeriodic)
		{
			const double* cv0, * cv1;
			int i0 = m_order[dir] - 2;
			int i1 = m_cv_count[dir] - 1;
			for (k = 0; k < m_cv_count[1 - dir]; k++)
			{
				cv0 = (dir) ? CV(k, i0) : CV(i0, k);
				cv1 = (dir) ? CV(k, i1) : CV(i1, k);
				for ( /*empty*/; i0 >= 0; i0--, i1--)
				{
					if (false == ON_PointsAreCoincident(m_dim, m_is_rat, cv0, cv1))
						return false;
					cv0 -= m_cv_stride[dir];
					cv1 -= m_cv_stride[dir];
				}
			}
		}
	}
	return bIsPeriodic;
}

double NurbsSurface::Knot(int dir, int i) const
{
	return (m_knot[dir ? 1 : 0]) ? m_knot[dir ? 1 : 0][i] : 0.0;
}

const double* NurbsSurface::Knot(int dir) const
{
	return m_knot[dir ? 1 : 0];
}

bool NurbsSurface::InsertKnot(int dir, double knot_value, int knot_multiplicity)
{
	bool rc = false;

	if ((dir == 0 || dir == 1) && IsValid() && knot_multiplicity > 0 && knot_multiplicity < Order(dir))
	{
		//ON_Interval domain = Domain(dir);
		//if (knot_value < domain.Min() || knot_value > domain.Max())
		//{
		//	ON_ERROR("ON_NurbsSurface::InsertKnot() knot_value not inside domain.");
		//}
		//else
		{
			//ReserveKnotCapacity(dir, CVCount(dir) + knot_multiplicity);
			ON_NurbsCurve crv;
			crv.m_knot = m_knot[dir];
			crv.m_knot_capacity = m_knot_capacity[dir];
			m_knot[dir] = 0;
			m_knot_capacity[dir] = 0;
			crv.ReserveKnotCapacity(CVCount(dir) + knot_multiplicity);
			ConvertToCurve(*this, dir, crv);
			rc = crv.InsertKnot(knot_value, knot_multiplicity);
			ConvertFromCurve(crv, dir, *this);
		}
	}

	return rc;
}


bool NurbsSurface::IsValid() const
{
	bool rc = false;

	if (m_dim <= 0)
	{
	}
	else if (m_cv == NULL)
	{
	}
	else
	{
		rc = true;
		int i;
		for (i = 0; i < 2 && rc; i++)
		{
			rc = false;
			if (m_order[i] < 2)
			{
			}
			else if (m_cv_count[i] < m_order[i])
			{
			}
			else if (m_knot[i] == NULL)
			{
			}
			//else if (!ON_IsValidKnotVector(m_order[i], m_cv_count[i], m_knot[i], text_log))
			//{
			//	if (text_log)
			//	{
			//		text_log->Print("ON_NurbsSurface.m_knot[%d] is not a valid knot vector.\n", i);
			//	}
			//}
			//else if (m_cv_stride[i] < CVSize())
			//{
			//	if (text_log)
			//	{
			//		text_log->Print("ON_NurbsSurface.m_cv_stride[%d]=%d is too small (should be >= %d).\n", i, m_cv_stride[i], CVSize());
			//	}
			//}
			else
				rc = true;
		}
		if (rc)
		{
			int a0 = CVSize();
			int a1 = m_cv_count[0] * a0;
			int b1 = CVSize();
			int b0 = m_cv_count[1] * b1;
			if (m_cv_stride[0] < a0 || m_cv_stride[1] < a1)
			{
				if (m_cv_stride[0] < b0 || m_cv_stride[1] < b1)
				{
					rc = false;
				}
			}
		}
	}

	return rc;
}

int NurbsSurface::CVSize() const
{
	return (m_is_rat) ? m_dim + 1 : m_dim;
}

int NurbsSurface::Order(int dir) const
{
	return m_order[dir ? 1 : 0];
}

bool NurbsSurface::SetCV(int i, int j, const PointT& point)
{
	bool rc = false;
	double* cv = CV(i, j);
	if (cv) {
		cv[0] = point.x;
		if (m_dim > 1) {
			cv[1] = point.y;
			if (m_dim > 2)
				cv[2] = point.z;

		}
		if (m_is_rat) {
			cv[m_dim] = 1.0;

		}
		rc = true;

	}
	return rc;
}

double* NurbsSurface::CV(int i, int j) const
{
	return (m_cv) ? (m_cv + i * m_cv_stride[0] + j * m_cv_stride[1]) : NULL;
}

int NurbsSurface::CVCount(void) const
{
	return m_cv_count[0] * m_cv_count[1];
}

int NurbsSurface::CVCount(int dir) const
{
	return m_cv_count[dir ? 1 : 0];
}

int NurbsSurface::Degree(int dir) const
{
	return (m_order[dir ? 1 : 0] >= 2) ? m_order[dir ? 1 : 0] - 1 : 0;
}

ON_Interval NurbsSurface::Domain(int dir) const
{
	ON_Interval d;
	if (dir) dir = 1;
	czx::ON_GetKnotVectorDomain(m_order[dir], m_cv_count[dir], m_knot[dir], &d.m_t[0], &d.m_t[1]);
	return d;
}

class myvec
{
public:
	int side;
	double hint;

	myvec(int side, double hint)
	{
		this->side = side;
		this->hint = hint;
	}
};

Vector2d NurbsSurface::inverseMappingBoundary(const Vector3d& pt, double& error, Vector3d& p, Vector3d& tu, Vector3d& tv, int maxSteps, double accuracy, bool quiet)
{

	Vector2d result;
	double min_err = 100.0;
	std::vector<myvec> ini_points;
	double err_tmp;
	Vector3d p_tmp, tu_tmp, tv_tmp;

	std::vector<double> elementsU = getElementVector(0);
	std::vector<double> elementsV = getElementVector(1);

	// NORTH - SOUTH
	for (unsigned i = 0; i < (elementsV.size() - 1); i++)
	{
		ini_points.push_back(myvec(pcl::on_nurbs::WEST, elementsV[i] + 0.5 * (elementsV[i + 1] - elementsV[i])));
		ini_points.push_back(myvec(pcl::on_nurbs::EAST, elementsV[i] + 0.5 * (elementsV[i + 1] - elementsV[i])));
	}

	// WEST - EAST
	for (unsigned i = 0; i < (elementsU.size() - 1); i++)
	{
		ini_points.push_back(myvec(pcl::on_nurbs::NORTH, elementsU[i] + 0.5 * (elementsU[i + 1] - elementsU[i])));
		ini_points.push_back(myvec(pcl::on_nurbs::SOUTH, elementsU[i] + 0.5 * (elementsU[i + 1] - elementsU[i])));
	}

	for (unsigned i = 0; i < ini_points.size(); i++)
	{

		Vector2d params = inverseMappingBoundary(pt, ini_points[i].side, ini_points[i].hint, err_tmp, p_tmp,
			tu_tmp, tv_tmp, maxSteps, accuracy, quiet);

		if (i == 0 || err_tmp < min_err)
		{
			min_err = err_tmp;
			result = params;
			p = p_tmp;
			tu = tu_tmp;
			tv = tv_tmp;
		}
	}

	error = min_err;
	return result;

}

Vector2d NurbsSurface::inverseMappingBoundary(const Vector3d& pt, int side, double hint, double& error, Vector3d& p, Vector3d& tu, Vector3d& tv, int maxSteps, double accuracy, bool quiet)
{

	double pointAndTangents[9];
	double current, delta;
	Vector3d r, t;
	Vector2d params;

	current = hint;

	std::vector<double> elementsU = getElementVector(0);
	std::vector<double> elementsV = getElementVector(1);
	double minU = elementsU[0];
	double minV = elementsV[0];
	double maxU = elementsU[elementsU.size() - 1];
	double maxV = elementsV[elementsV.size() - 1];

	for (int k = 0; k < maxSteps; k++)
	{

		switch (side)
		{

		case pcl::on_nurbs::WEST:

			params(0) = minU;
			params(1) = current;
			Evaluate(minU, current, 1, 3, pointAndTangents);
			p(0) = pointAndTangents[0];
			p(1) = pointAndTangents[1];
			p(2) = pointAndTangents[2];
			tu(0) = pointAndTangents[3];
			tu(1) = pointAndTangents[4];
			tu(2) = pointAndTangents[5];
			tv(0) = pointAndTangents[6];
			tv(1) = pointAndTangents[7];
			tv(2) = pointAndTangents[8];

			t = tv; // use tv

			break;
		case pcl::on_nurbs::SOUTH:

			params(0) = current;
			params(1) = maxV;
			Evaluate(current, maxV, 1, 3, pointAndTangents);
			p(0) = pointAndTangents[0];
			p(1) = pointAndTangents[1];
			p(2) = pointAndTangents[2];
			tu(0) = pointAndTangents[3];
			tu(1) = pointAndTangents[4];
			tu(2) = pointAndTangents[5];
			tv(0) = pointAndTangents[6];
			tv(1) = pointAndTangents[7];
			tv(2) = pointAndTangents[8];

			t = tu; // use tu

			break;
		case pcl::on_nurbs::EAST:

			params(0) = maxU;
			params(1) = current;
			Evaluate(maxU, current, 1, 3, pointAndTangents);
			p(0) = pointAndTangents[0];
			p(1) = pointAndTangents[1];
			p(2) = pointAndTangents[2];
			tu(0) = pointAndTangents[3];
			tu(1) = pointAndTangents[4];
			tu(2) = pointAndTangents[5];
			tv(0) = pointAndTangents[6];
			tv(1) = pointAndTangents[7];
			tv(2) = pointAndTangents[8];

			t = tv; // use tv

			break;
		case pcl::on_nurbs::NORTH:

			params(0) = current;
			params(1) = minV;
			Evaluate(current, minV, 1, 3, pointAndTangents);
			p(0) = pointAndTangents[0];
			p(1) = pointAndTangents[1];
			p(2) = pointAndTangents[2];
			tu(0) = pointAndTangents[3];
			tu(1) = pointAndTangents[4];
			tu(2) = pointAndTangents[5];
			tv(0) = pointAndTangents[6];
			tv(1) = pointAndTangents[7];
			tv(2) = pointAndTangents[8];

			t = tu; // use tu

			break;
		default:
			throw std::runtime_error("[FittingSurface::inverseMappingBoundary] ERROR: Specify a boundary!");

		}

		r(0) = pointAndTangents[0] - pt(0);
		r(1) = pointAndTangents[1] - pt(1);
		r(2) = pointAndTangents[2] - pt(2);

		delta = -0.5 * r.dot(t) / t.dot(t);

		if (fabs(delta) < accuracy)
		{

			error = r.norm();
			return params;

		}
		else
		{

			current = current + delta;

			bool stop = false;

			switch (side)
			{

			case pcl::on_nurbs::WEST:
			case pcl::on_nurbs::EAST:
				if (current < minV)
				{
					params(1) = minV;
					stop = true;
				}
				else if (current > maxV)
				{
					params(1) = maxV;
					stop = true;
				}

				break;

			case pcl::on_nurbs::NORTH:
			case pcl::on_nurbs::SOUTH:
				if (current < minU)
				{
					params(0) = minU;
					stop = true;
				}
				else if (current > maxU)
				{
					params(0) = maxU;
					stop = true;
				}

				break;
			}

			if (stop)
			{
				error = r.norm();
				return params;
			}

		}

	}

	error = r.norm();
	if (!quiet)
		printf(
			"[FittingSurface::inverseMappingBoundary] Warning: Method did not converge! (residual: %f, delta: %f, params: %f %f)\n",
			error, delta, params(0), params(1));

	return params;
}

bool NurbsSurface::GetCV(int i, int j, PointT& point) const
{
	bool rc = false;
	const double* cv = CV(i, j);
	if (cv) {
		if (m_is_rat) {
			if (cv[m_dim] != 0.0) {
				const double w = 1.0 / cv[m_dim];
				point.x = cv[0] * w;
				point.y = (m_dim > 1) ? cv[1] * w : 0.0;
				point.z = (m_dim > 2) ? cv[2] * w : 0.0;
				rc = true;

			}

		}
		else {
			point.x = cv[0];
			point.y = (m_dim > 1) ? cv[1] : 0.0;
			point.z = (m_dim > 2) ? cv[2] : 0.0;
			rc = true;

		}

	}
	return rc;
}












bool czx::ON_IsKnotVectorClamped(
	int order,
	int cv_count,
	const double* knot,
	int end // (default = 2) 0 = left end, 1 = right end, 2 = both
)

{
	if (order <= 1 || cv_count < order || !knot || end < 0 || end > 2)
		return false;
	bool rc = true;
	if ((end == 0 || end == 2) && knot[0] != knot[order - 2])
		rc = false;
	if ((end == 1 || end == 2) && knot[cv_count - 1] != knot[order + cv_count - 3])
		rc = false;
	return rc;
}

bool ON_PointsAreCoincident(
	int dim,
	int is_rat,
	const double* pointA,
	const double* pointB
)
{
	double d, a, b, wa, wb;

	if (dim < 1 || 0 == pointA || 0 == pointB)
		return false;

	if (is_rat)
	{
		wa = pointA[dim];
		wb = pointB[dim];
		if (0.0 == wa || 0.0 == wb)
		{
			if (0.0 == wa && 0.0 == wb)
				return ON_PointsAreCoincident(dim, 0, pointA, pointB);
			return false;
		}
		while (dim--)
		{
			a = *pointA++ / wa;
			b = *pointB++ / wb;
			d = fabs(a - b);
			if (d <= ON_ZERO_TOLERANCE)
				continue;
			if (d <= (fabs(a) + fabs(b)) * ON_RELATIVE_TOLERANCE)
				continue;
			return false;
		}
	}
	else
	{
		while (dim--)
		{
			a = *pointA++;
			b = *pointB++;
			d = fabs(a - b);
			if (d <= ON_ZERO_TOLERANCE)
				continue;
			if (d <= (fabs(a) + fabs(b)) * ON_RELATIVE_TOLERANCE)
				continue;
			return false;
		}
	}

	return true;
}

int czx::ON_ComparePoint( // returns 
								  // -1: first < second
								  //  0: first == second
								  // +1: first > second
	int dim,
	bool is_rat,
	const double* pointA,
	const double* pointB
)
{
	const double wA = (is_rat && pointA[dim] != 0.0) ? 1.0 / pointA[dim] : 1.0;
	const double wB = (is_rat && pointB[dim] != 0.0) ? 1.0 / pointB[dim] : 1.0;
	double a, b, tol;
	int i;
	for (i = 0; i < dim; i++) {
		a = wA * *pointA++;
		b = wB * *pointB++;
		tol = (fabs(a) + fabs(b)) * ON_RELATIVE_TOLERANCE;
		if (tol < ON_ZERO_TOLERANCE)
			tol = ON_ZERO_TOLERANCE;
		if (a < b - tol)
			return -1;
		if (b < a - tol)
			return 1;
		if (wA < wB - ON_SQRT_EPSILON)
			return -1;
		if (wB < wA - ON_SQRT_EPSILON)
			return -1;

	}
	return 0;
}

int ON_ComparePointList( // returns 
									// -1: first < second
									//  0: first == second
									// +1: first > second
	int dim,
	bool is_rat,
	int point_count,
	int point_strideA,
	const double* pointA,
	int point_strideB,
	const double* pointB
)
{
	int i, rc = 0, rc1 = 0;
	bool bDoSecondCheck = (1 == is_rat && dim <= 3 && point_count > 0
		&& ON_IsValid(pointA[dim]) && 0.0 != pointA[dim]
		&& ON_IsValid(pointB[dim]) && 0.0 != pointB[dim]
		);
	const double wA = bDoSecondCheck ? pointA[dim] : 1.0;
	const double wB = bDoSecondCheck ? pointB[dim] : 1.0;
	const double wAtol = wA * ON_ZERO_TOLERANCE;
	const double wBtol = wB * ON_ZERO_TOLERANCE;
	double A[3] = { 0.0,0.0,0.0 };
	double B[3] = { 0.0,0.0,0.0 };
	const size_t AB_size = dim * sizeof(A[0]);

	for (i = 0; i < point_count && !rc; i++)
	{
		rc = czx::ON_ComparePoint(dim, is_rat, pointA, pointB);
		if (rc && bDoSecondCheck
			&& fabs(wA - pointA[dim]) <= wAtol
			&& fabs(wB - pointB[dim]) <= wBtol)
		{
			if (!rc1)
				rc1 = rc;
			memcpy(A, pointA, AB_size);
			A[0] /= pointA[dim]; A[1] /= pointA[dim]; A[2] /= pointA[dim];
			memcpy(B, pointB, AB_size);
			B[0] /= pointB[dim]; B[1] /= pointB[dim]; B[2] /= pointB[dim];
			rc = (0 == czx::ON_ComparePoint(dim, 0, A, B)) ? 0 : rc1;
		}
		pointA += point_strideA;
		pointB += point_strideB;
	}

	return rc;
}

bool czx::ON_IsPointGridClosed(
		int dim,
		bool is_rat,
		int point_count0, int point_count1,
		int point_stride0, int point_stride1,
		const double* p,
		int dir
	)
{
	bool rc = false;
	if (point_count0 > 0 && point_count1 > 0 && p != NULL) {
		int count, stride;
		const double* p0;
		const double* p1;
		if (dir) {
			p0 = p;
			p1 = p + (point_count1 - 1) * point_stride1;
			count = point_count0;
			stride = point_stride0;

		}
		else {
			p0 = p;
			p1 = p + (point_count0 - 1) * point_stride0;
			count = point_count1;
			stride = point_stride1;

		}
		rc = (0 == ON_ComparePointList(dim, is_rat, count, stride, p0, stride, p1)) ? true : false;

	}
	return rc;
}


int ON_NurbsSpanIndex(
	int order,          // (>=2)
	int cv_count,
	const double* knot, // knot[] array or length ON_KnotCount(order,cv_count)
	double t,           // evaluation parameter
	int side,           // side 0 = default, -1 = from below, +1 = from above
	int hint            // hint (or 0 if no hint available)
)
{
	int j, len;

	// shift knot so that domain is knot[0] to knot[len]
	knot += (order - 2);
	len = cv_count - order + 2;

	// see if hint helps 
	if (hint > 0 && hint < len - 1) {
		while (hint > 0 && knot[hint - 1] == knot[hint]) hint--;
		if (hint > 0) {
			// have knot[hint-1] < knot[hint]
			if (t < knot[hint]) {
				len = hint + 1;
				hint = 0;

			}
			else {
				if (side < 0 && t == knot[hint])
					hint--;
				knot += hint;
				len -= hint;

			}

		}

	}
	else
		hint = 0;

	j = czx::ON_SearchMonotoneArray(knot, len, t);
	if (j < 0)
		j = 0;
	else if (j >= len - 1)
		j = len - 2;
	else if (side < 0) {
		// if user wants limit from below and t = an internal knot,
			// back up to prevous span
		while (j > 0 && t == knot[j])
			j--;

	}
	return (j + hint);
}

bool ON_IsKnotVectorPeriodic(
	int order,
	int cv_count,
	const double* knot
)

{
	double tol;
	const double* k1;
	int i;

	if (order < 2 || cv_count < order || !knot) {
		cout << ("ON_IsKnotVectorPeriodic(): illegal input") << endl;
		return false;

	}

	if (order == 2)
		return false; // convention is that degree 1 curves cannot be periodic.

	if (order <= 4) {
		if (cv_count < order + 2)
			return false;

	}
	else if (cv_count < 2 * order - 2) {
		return false;

	}

	tol = fabs(knot[order - 1] - knot[order - 3]) * ON_SQRT_EPSILON;
	if (tol < fabs(knot[cv_count - 1] - knot[order - 2]) * ON_SQRT_EPSILON)
		tol = fabs(knot[cv_count - 1] - knot[order - 2]) * ON_SQRT_EPSILON;
	k1 = knot + cv_count - order + 1;
	i = 2 * (order - 2);
	while (i--) {
		if (fabs(knot[1] - knot[0] + k1[0] - k1[1]) > tol)
			return false;
		knot++; k1++;

	}
	return true;
}

int czx::ON_SearchMonotoneArray(const double* array, int length, double t)
	/*****************************************************************************
	Find interval in an increasing array of doubles

	INPUT:
	array
		A monotone increasing (array[i] <= array[i+1]) array of length doubles.
	length (>=1)
		number of doubles in array
	t
		parameter
	OUTPUT:
	ON_GetdblArrayIndex()
		-1:         t < array[0]
		i:         (0 <= i <= length-2) array[i] <= t < array[i+1]
		length-1:  t == array[length-1]
		length:    t  > array[length-1]
	COMMENTS:
	If length < 1 or array is not monotone increasing, you will get a meaningless
	answer and may crash your program.
	EXAMPLE:
	// Given a "t", find the knots that define the span used to evaluate a
	// nurb at t; i.e., find "i" so that
	// knot[i] <= ... <= knot[i+order-2]
	//   <= t < knot[i+order-1] <= ... <= knot[i+2*(order-1)-1]
	i = ON_GetdblArrayIndex(knot+order-2,cv_count-order+2,t);
	if (i < 0) i = 0; else if (i > cv_count - order) i = cv_count - order;
	RELATED FUNCTIONS:
	ON_
	ON_
	*****************************************************************************/

{
	int
		i, i0, i1;

	length--;

	/* Since t is frequently near the ends and bisection takes the
	* longest near the ends, trap those cases here.
	*/
	if (t < array[0])
		return -1;
	if (t >= array[length])
		return (t > array[length]) ? length + 1 : length;
	if (t < array[1])
		return 0;
	if (t >= array[length - 1])
		return (length - 1);


	i0 = 0;
	i1 = length;
	while (array[i0] == array[i0 + 1]) i0++;
	while (array[i1] == array[i1 - 1]) i1--;
	/* From now on we have
	*  1.) array[i0] <= t < array[i1]
	*  2.) i0 <= i < i1.
	* When i0+1 == i1, we have array[i0] <= t < array[i0+1]
	* and i0 is the answer we seek.
	*/
	while (i0 + 1 < i1) {
		i = (i0 + i1) >> 1;
		if (t < array[i]) {
			i1 = i;
			while (array[i1] == array[i1 - 1]) i1--;

		}
		else {
			i0 = i;
			while (array[i0] == array[i0 + 1]) i0++;

		}

	}

	return i0;
}

bool ON_EvaluateNurbsSurfaceSpan(
	int dim,
	int is_rat,
	int order0, int order1,
	const double* knot0,
	const double* knot1,
	int cv_stride0, int cv_stride1,
	const double* cv0, // cv at "lower left" of bispan
	int der_count,
	double t0, double t1,
	int v_stride,
	double* v      // returns values
)
{
	const int der_count0 = (der_count >= order0) ? order0 - 1 : der_count;
	const int der_count1 = (der_count >= order1) ? order1 - 1 : der_count;
	const double* cv;

	double* N_0, * N_1, * P0, * P;
	double c;
	int d1max, d, d0, d1, i, j, j0, j1, Pcount, Psize;

	const int cvdim = (is_rat) ? dim + 1 : dim;
	const int dcv1 = cv_stride1 - cvdim;

	// get work space memory
	i = order0 * order0;
	j = order1 * order1;
	Pcount = ((der_count + 1) * (der_count + 2)) >> 1;
	Psize = cvdim << 3;
	N_0 = (double*)alloca(((i + j) << 3) + Pcount * Psize);
	N_1 = N_0 + i;
	P0 = N_1 + j;
	memset(P0, 0, Pcount * Psize);

	/* evaluate basis functions */
	ON_EvaluateNurbsBasis(order0, knot0, t0, N_0);
	ON_EvaluateNurbsBasis(order1, knot1, t1, N_1);
	if (der_count0) {
		// der_count0 > 0 iff der_count1 > 0 
		ON_EvaluateNurbsBasisDerivatives(order0, knot0, der_count0, N_0);
		ON_EvaluateNurbsBasisDerivatives(order1, knot1, der_count1, N_1);

	}

	// compute point
	P = P0;
	for (j0 = 0; j0 < order0; j0++) {
		cv = cv0 + j0 * cv_stride0;
		for (j1 = 0; j1 < order1; j1++) {
			c = N_0[j0] * N_1[j1];
			j = cvdim;
			while (j--)
				*P++ += c * *cv++;
			P -= cvdim;
			cv += dcv1;

		}

	}

	if (der_count > 0) {
		// compute first derivatives
		P += cvdim; // step over point
		for (j0 = 0; j0 < order0; j0++) {
			cv = cv0 + j0 * cv_stride0;
			for (j1 = 0; j1 < order1; j1++) {
				// "Ds"
				c = N_0[j0 + order0] * N_1[j1];
				j = cvdim;
				while (j--)
					*P++ += c * *cv++;
				cv -= cvdim;

				// "Dt"
				c = N_0[j0] * N_1[j1 + order1];
				j = cvdim;
				while (j--)
					*P++ += c * *cv++;
				P -= cvdim;
				P -= cvdim;

				cv += dcv1;

			}

		}

		if (der_count > 1) {
			// compute second derivatives
			P += cvdim; // step over "Ds"
			P += cvdim; // step over "Dt"
			if (der_count0 + der_count1 > 1) {
				// compute "Dss"
				for (j0 = 0; j0 < order0; j0++) {
					// P points to first coordinate of Dss
					cv = cv0 + j0 * cv_stride0;
					for (j1 = 0; j1 < order1; j1++) {
						if (der_count0 > 1) {
							// "Dss"
							c = N_0[j0 + 2 * order0] * N_1[j1];
							j = cvdim;
							while (j--)
								*P++ += c * *cv++;
							cv -= cvdim;

						}
						else {
							P += cvdim; // Dss = 0

						}

						// "Dst"
						c = N_0[j0 + order0] * N_1[j1 + order1];
						j = cvdim;
						while (j--)
							*P++ += c * *cv++;
						cv -= cvdim;

						if (der_count1 > 1) {
							// "Dtt"
							c = N_0[j0] * N_1[j1 + 2 * order1];
							j = cvdim;
							while (j--)
								*P++ += c * *cv++;
							cv -= cvdim;
							P -= cvdim;

						}

						P -= cvdim;
						P -= cvdim;
						cv += cv_stride1;

					}

				}

			}

			if (der_count > 2)
			{
				// 12 February 2004 Dale Lear
					//     Bug fix for d^n/ds^n when n >= 3
					// compute higher derivatives in slower generic loop
				for (d = 3; d <= der_count; d++)
				{
					P += d * cvdim; // step over (d-1)th derivatives
					d1max = (d > der_count1) ? der_count1 : d;
					for (j0 = 0; j0 < order0; j0++)
					{
						cv = cv0 + j0 * cv_stride0;
						for (j1 = 0; j1 < order1; j1++)
						{
							for (d0 = d, d1 = 0;
								d0 > der_count0 && d1 <= d1max;
								d0--, d1++)
							{
								// partial with respect to "s" is zero
								P += cvdim;
							}
							for ( /*empty*/; d1 <= d1max; d0--, d1++)
							{
								c = N_0[j0 + d0 * order0] * N_1[j1 + d1 * order1];
								j = cvdim;
								while (j--)
									*P++ += c * *cv++;
								cv -= cvdim;
							}
							// remaining partials with respect to "t" are zero
								// - reset and add contribution from the next cv
							P -= d1 * cvdim;
							cv += cv_stride1;
						}
					}
				}
			}


		}

	}

	if (is_rat) {
		ON_EvaluateQuotientRule2(dim, der_count, cvdim, P0);
		Psize -= 8;

	}
	for (i = 0; i < Pcount; i++) {
		memcpy(v, P0, Psize);
		v += v_stride;
		P0 += cvdim;

	}

	return true;
}


bool ON_EvaluateNurbsBasis(int order, const double* knot,
	double t, double* N)
{
	/*****************************************************************************
	Evaluate B-spline basis functions

	INPUT:
	  order >= 1
		d = degree = order - 1
	  knot[]
		array of length 2*d.
		Generally, knot[0] <= ... <= knot[d-1] < knot[d] <= ... <= knot[2*d-1].
	  N[]
		array of length order*order

	OUTPUT:
	  If "N" were declared as double N[order][order], then

					 k
		N[d-k][i] = N (t) = value of i-th degree k basis function.
					 i
	  where 0 <= k <= d and k <= i <= d.

			In particular, N[0], ..., N[d] - values of degree d basis functions.
	  The "lower left" triangle is not initialized.

	  Actually, the above is true when knot[d-1] <= t < knot[d].  Otherwise, the
	  value returned is the value of the polynomial that agrees with N_i^k on the
	  half open domain [ knot[d-1], knot[d] )

	COMMENTS:
	  If a degree d NURBS has n control points, then the TL knot vector has
	  length d+n-1. ( Most literature, including DeBoor and The NURBS Book,
	  duplicate the TL start and end knot and have knot vectors of length
	  d+n+1. )

	  Assume C is a B-spline of degree d (order=d+1) with n control vertices
	  (n>=d+1) and knot[] is its knot vector.  Then

		C(t) = Sum( 0 <= i < n, N_{i}(t) * C_{i} )

	  where N_{i} are the degree d b-spline basis functions and C_{i} are the control
	  vertices.  The knot[] array length d+n-1 and satisfies

		knot[0] <= ... <= knot[d-1] < knot[d]
		knot[n-2] < knot[n-1] <= ... <= knot[n+d-2]
		knot[i] < knot[d+i] for 0 <= i < n-1
		knot[i] <= knot[i+1] for 0 <= i < n+d-2

	  The domain of C is [ knot[d-1], knot[n-1] ].

	  The support of N_{i} is [ knot[i-1], knot[i+d] ).

	  If d-1 <= k < n-1 and knot[k] <= t < knot[k+1], then
	  N_{i}(t) = 0 if i <= k-d
			   = 0 if i >= k+2
			   = B[i-k+d-1] if k-d+1 <= i <= k+1, where B[] is computed by the call
				 TL_EvNurbBasis( d+1, knot+k-d+1, t, B );

	  If 0 <= j < n-d, 0 <= m <= d, knot[j+d-1] <= t < knot[j+d], and B[] is
	  computed by the call

		TL_EvNurbBasis( d+1, knot+j, t, B ),

	  then

		N_{j+m}(t) = B[m].

	EXAMPLE:
	REFERENCE:
	  The NURBS book
	RELATED FUNCTIONS:
	  TL_EvNurbBasis
	  TL_EvNurbBasisDer
	*****************************************************************************/
	register double a0, a1, x, y;
	const double* k0;
	double* t_k, * k_t, * N0;
	const int d = order - 1;
	register int j, r;

	t_k = (double*)alloca(d << 4);
	k_t = t_k + d;

	if (knot[d - 1] == knot[d]) {
		/* value is defined to be zero on empty spans */
		memset(N, 0, order * order * sizeof(*N));
		return true;

	}

	N += order * order - 1;
	N[0] = 1.0;
	knot += d;
	k0 = knot - 1;

	for (j = 0; j < d; j++) {
		N0 = N;
		N -= order + 1;
		t_k[j] = t - *k0--;
		k_t[j] = *knot++ - t;

		x = 0.0;
		for (r = 0; r <= j; r++) {
			a0 = t_k[j - r];
			a1 = k_t[r];
			y = N0[r] / (a0 + a1);
			N[r] = x + a1 * y;
			x = a0 * y;

		}

		N[r] = x;

	}

	// 16 September 2003 Dale Lear (at Chuck's request)
	//   When t is at an end knot, do a check to
	//   get exact values of basis functions.
	//   The problem being that a0*y above can
	//   fail to be one by a bit or two when knot
	//   values are large.
	x = 1.0 - ON_SQRT_EPSILON;
	if (N[0] > x)
	{
		if (N[0] != 1.0 && N[0] < 1.0 + ON_SQRT_EPSILON)
		{
			r = 1;
			for (j = 1; j <= d && r; j++)
			{
				if (N[j] != 0.0)
					r = 0;
			}
			if (r)
				N[0] = 1.0;
		}
	}
	else if (N[d] > x)
	{
		if (N[d] != 1.0 && N[d] < 1.0 + ON_SQRT_EPSILON)
		{
			r = 1;
			for (j = 0; j < d && r; j++)
			{
				if (N[j] != 0.0)
					r = 0;
			}
			if (r)
				N[d] = 1.0;
		}
	}

	return true;
}

bool ON_EvaluateQuotientRule2(int dim, int der_count, int v_stride, double* v)
{
	double
		F, Fs, Ft, ws, wt, wss, wtt, wst, * f, * x;
	int
		i, j, n, q, ii, jj, Fn;

	// comment notation:
	//  X = value of numerator
	//  W = value of denominator
	//  F = X/W
	//  Xs = partial w.r.t. 1rst parameter
	//  Xt = partial w.r.t. 2nd parameter 
	//  ...
	// 

		  // divide everything by the weight
	F = v[dim];
	if (F == 0.0)
		return false;
	F = 1.0 / F;
	if (v_stride > dim + 1)
	{
		i = ((der_count + 1) * (der_count + 2) >> 1);
		x = v;
		j = dim + 1;
		q = v_stride - j;
		while (i--)
		{
			jj = j;
			while (jj--)
				*x++ *= F;
			x += q;
		}
	}
	else
	{
		i = (((der_count + 1) * (der_count + 2)) >> 1) * v_stride;
		x = v;
		while (i--) *x++ *= F;
	}

	if (der_count)
	{
		// first derivatives
		f = v;                    // f = F
		x = v + v_stride;         // x = Xs/w, x[v_stride] = Xt/w
		ws = -x[dim];             // ws = -Ws/w
		wt = -x[dim + v_stride];    // wt = -Wt/w
		j = dim;
		while (j--)
		{
			F = *f++;
			*x += ws * F;
			x[v_stride] += wt * F;
			x++;
		}

		if (der_count > 1)
		{
			// 2nd derivatives
			f += (v_stride - dim);           // f = Fs, f[cvdim] = Ft 
			x = v + 3 * v_stride;     // x = Xss, x[v_stride] = Xst, x[2*v_stride] = Xtt
			wss = -x[dim];          // wss = -wss/W
			wst = -x[v_stride + dim]; // wst = -Wst/W 
			n = 2 * v_stride;
			wtt = -x[n + dim];        // wtt = -Wtt/w
			j = dim;
			while (j--)
			{
				F = *v++;
				Ft = f[v_stride];
				Fs = *f++;
				*x += wss * F + 2.0 * ws * Fs;     // Dss
				x[v_stride] += wst * F + wt * Fs + ws * Ft; // Dst
				x[n] += wtt * F + 2.0 * wt * Ft;     // Dtt
				x++;
			}

			if (der_count > 2)
			{
				// general loop for higher derivatives 
				v -= dim; // restore v pointer to input value
				f = v + 6 * v_stride;    // f = Xsss
				for (n = 3; n <= der_count; n++)
				{
					for (j = 0; j <= n; j++)
					{
						// f = Ds^i Dt^j
							// 13 Jan 2005 Dale Lear bug fix - added missing a binomial coefficients.
						i = n - j;
						for (ii = 0; ii <= i; ii++)
						{
							ws = ON_BinomialCoefficient(ii, i - ii);
							for (jj = ii ? 0 : 1; jj <= j; jj++)
							{
								q = ii + jj;
								Fn = ((q * (q + 1)) / 2 + jj) * v_stride + dim;
								// wt = -(i choose ii)*(j choose jj)*W(ii,jj)
								wt = -ws * ON_BinomialCoefficient(jj, j - jj) * v[Fn];
								q = n - q;
								Fn = ((q * (q + 1)) / 2 + j - jj) * v_stride; // X(i-ii,j-jj) = v[Fn]
								for (q = 0; q < dim; q++)
									f[q] += wt * v[Fn + q];
							}
						}
						f += v_stride;
					}
				}
			}
		}
	}

	return true;
}


bool ON_EvaluateNurbsBasisDerivatives(int order, const double* knot,
	int der_count, double* N)
{
	/* INPUT:
	 *   Results of the call
	 *      TL_EvNurbBasis( order, knot, t, N );  (initializes N[] )
	 *   are sent to
	 *      TL_EvNurbBasisDer( order, knot, der_count, N ),
	 *   where 1 <= der_count < order
	 *
	 * OUTPUT:
*  If "N" were declared as double N[order][order], then
	  *
*                                    d
*    N[d-k][i] = k-th derivative of N (t)
*                                    i
*
	  *  where 0 <= k <= d and 0 <= i <= d.
	  *
	  * In particular,
	  *   N[0], ..., N[d] - values of degree d basis functions.
	  *   N[order], ..., N[order_d] - values of first derivative.
	  *
* Actually, the above is true when knot[d-1] <= t < knot[d].  Otherwise, the
* values returned are the values of the polynomials that agree with N_i^k on the
* half open domain [ knot[d-1], knot[d] )
	  *
	  * Ref: The NURBS Book
	  */
	double dN, c;
	const double* k0, * k1;
	double* a0, * a1, * ptr, ** dk;
	int i, j, k, jmax;

	const int d = order - 1;
	const int Nstride = -der_count * order;

	/* workspaces for knot differences and coefficients
	 *
	 * a0[] and a1[] have order doubles
	 *
	 * dk[0] = array of d knot differences
	 * dk[1] = array of (d-1) knot differences
	 *
	 * dk[der_count-1] = 1.0/(knot[d] - knot[d-1])
	 * dk[der_count] = dummy pointer to make loop efficient
	 */
	dk = (double**)alloca((der_count + 1) << 3); /* << 3 in case pointers are 8 bytes long */
	a0 = (double*)alloca((order * (2 + ((d + 1) >> 1))) << 3); /* d for a0, d for a1, d*order/2 for dk[]'s and slop to avoid /2 */
	a1 = a0 + order;

	/* initialize reciprocal of knot differences */
	dk[0] = a1 + order;
	for (k = 0; k < der_count; k++) {
		j = d - k;
		k0 = knot++;
		k1 = k0 + j;
		for (i = 0; i < j; i++)
			dk[k][i] = 1.0 / (*k1++ - *k0++);
		dk[k + 1] = dk[k] + j;

	}
	dk--;
	/* dk[1] = 1/{t[d]-t[0], t[d+1]-t[1], ..., t[2d-2] - t[d-2], t[2d-1] - t[d-1]}
	 *       = diffs needed for 1rst derivative
	 * dk[2] = 1/{t[d]-t[1], t[d+1]-t[2], ..., t[2d-2] - t[d-1]}
	 *       = diffs needed for 2nd derivative
	 * ...
	 * dk[d] = 1/{t[d] - t[d-1]}
	 *       = diff needed for d-th derivative
	 *
	 * d[k][n] = 1.0/( t[d+n] - t[k-1+n] )
	 */

	N += order;
	/* set N[0] ,..., N[d] = 1rst derivatives,
	 * N[order], ..., N[order+d] = 2nd, etc.
	 */
	for (i = 0; i < order; i++) {
		a0[0] = 1.0;
		for (k = 1; k <= der_count; k++) {
			/* compute k-th derivative of N_i^d up to d!/(d-k)! scaling factor */
			dN = 0.0;
			j = k - i;
			if (j <= 0) {
				dN = (a1[0] = a0[0] * dk[k][i - k]) * N[i];
				j = 1;

			}
			jmax = d - i;
			if (jmax < k) {
				while (j <= jmax) {
					dN += (a1[j] = (a0[j] - a0[j - 1]) * dk[k][i + j - k]) * N[i + j];
					j++;

				}

			}
			else {
				/* sum j all the way to j = k */
				while (j < k) {
					dN += (a1[j] = (a0[j] - a0[j - 1]) * dk[k][i + j - k]) * N[i + j];
					j++;

				}
				dN += (a1[k] = -a0[k - 1] * dk[k][i]) * N[i + k];

			}

			/* d!/(d-k)!*dN = value of k-th derivative */
			N[i] = dN;
			N += order;
			/* a1[] s for next derivative = linear combination
			 * of a[]s used to compute this derivative.
			 */
			ptr = a0; a0 = a1; a1 = ptr;

		}
		N += Nstride;

	}

	/* apply d!/(d-k)! scaling factor */
	dN = c = (double)d;
	k = der_count;
	while (k--) {
		i = order;
		while (i--)
			*N++ *= c;
		dN -= 1.0;
		c *= dN;

	}
	return true;
}

Eigen::Vector3d computeMean(const Eigen::MatrixXd& data)
{
	Eigen::Vector3d u(0.0, 0.0, 0.0);

	unsigned s = unsigned(data.cols());
	double ds = 1.0 / s;

	for (unsigned i = 0; i < s; i++)
		u += (data.col(i) * ds);

	return u;
}

void pca(const Eigen::MatrixXd data, Eigen::Vector3d& mean, Eigen::Matrix3d& eigenvectors,
		Eigen::Vector3d& eigenvalues)
{
	mean = computeMean(data);

	unsigned s = unsigned(data.cols());

	Eigen::MatrixXd Q(3, s);

	for (unsigned i = 0; i < s; i++)
		Q.col(i) << (data.col(i) - mean);

	Eigen::Matrix3d C = Q * Q.transpose();

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(C);
	if (eigensolver.info() != Success)
	{
		printf("[NurbsTools::pca] Can not find eigenvalues.\n");
		abort();
	}

	for (int i = 0; i < 3; ++i)
	{
		eigenvalues(i) = eigensolver.eigenvalues() (2 - i);
		if (i == 2)
			eigenvectors.col(2) = eigenvectors.col(0).cross(eigenvectors.col(1));
		else
			eigenvectors.col(i) = eigensolver.eigenvectors().col(2 - i);
	}
}

void onfree(void* memblock)
{
	if (0 != memblock)
		free(memblock);
}

static void ConvertFromCurve(ON_NurbsCurve& crv, int dir, NurbsSurface& srf)
{
	// DO NOT MAKE THIS FUNCTION PUBLIC - IT IS DELICATE AND DEDICATED TO USE IN THIS FILE

	if (dir)
		dir = 1;
	const int Sdim = srf.CVSize();

	srf.m_order[dir] = crv.m_order;
	srf.m_cv_count[dir] = crv.m_cv_count;
	srf.m_cv_stride[dir] = crv.m_cv_stride;
	srf.m_cv_stride[1 - dir] = Sdim;

	if (crv.m_cv)
	{
		if (srf.m_cv
			&& crv.m_cv != srf.m_cv
			&& srf.m_cv_capacity > 0
			&& srf.m_cv_capacity < crv.m_cv_stride * crv.m_cv_count)
		{
			// discard surface cvs because there isn't enough room
			onfree(srf.m_cv);
			srf.m_cv = 0;
			srf.m_cv_capacity = 0;
		}

		if (srf.m_cv)
		{
			// use existing surface cvs
			memcpy(srf.m_cv, crv.m_cv, crv.m_cv_stride * crv.m_cv_count * sizeof(*srf.m_cv));
		}
		else
		{
			// move curve cvs to surface
			srf.m_cv = crv.m_cv;
			srf.m_cv_capacity = crv.m_cv_capacity;
			crv.m_cv = 0;
			crv.m_cv_capacity = 0;
		}

		crv.m_cv_stride = 0;
	}

	if (crv.m_knot && crv.m_knot != srf.m_knot[dir])
	{
		if (srf.m_knot_capacity[dir] > 0)
		{
			onfree(srf.m_knot[dir]);
			srf.m_knot[dir] = 0;
			srf.m_knot_capacity[dir] = 0;
		}
		srf.m_knot[dir] = crv.m_knot;
		srf.m_knot_capacity[dir] = crv.m_knot_capacity;
		crv.m_knot = 0;
		crv.m_knot_capacity = 0;
	}
}

static void ConvertToCurve(NurbsSurface& srf, int dir, ON_NurbsCurve& crv)
{
	// DO NOT MAKE THIS FUNCTION PUBLIC - IT IS DELICATE AND DEDICATED TO USE IN THIS FILE

	if (dir)
		dir = 1;
	const int Sdim = srf.CVSize();
	const int n = srf.CVCount(1 - dir);
	const int Ndim = Sdim * n;
	const int knot_count = srf.KnotCount(dir);
	int i, j;
	double* Ncv;
	const double* Scv;

	crv.m_dim = Ndim;
	crv.m_is_rat = 0;
	crv.m_order = srf.Order(dir);
	crv.m_cv_count = srf.CVCount(dir);
	crv.m_cv_stride = crv.m_dim;
	crv.ReserveCVCapacity(srf.CVCount(dir) * Ndim);
	crv.ReserveKnotCapacity(srf.KnotCount(dir));

	if (crv.m_knot != srf.m_knot[dir] && srf.m_knot[dir])
	{
		memcpy(crv.m_knot, srf.m_knot[dir], knot_count * sizeof(crv.m_knot[0]));
	}

	if (crv.m_cv != srf.m_cv && srf.m_cv)
	{
		if (dir) {
			for (i = 0; i < crv.m_cv_count; i++) {
				Ncv = crv.CV(i);
				for (j = 0; j < n; j++) {
					Scv = srf.CV(j, i);
					memcpy(Ncv, Scv, Sdim * sizeof(*Ncv));
					Ncv += Sdim;

				}

			}

		}
		else {
			for (i = 0; i < crv.m_cv_count; i++) {
				Ncv = crv.CV(i);
				for (j = 0; j < n; j++) {
					Scv = srf.CV(i, j);
					memcpy(Ncv, Scv, Sdim * sizeof(*Ncv));
					Ncv += Sdim;

				}

			}

		}
	}
}
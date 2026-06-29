#pragma once

#include<Eigen/Dense>
#include"czxTool.h"
#include<pcl/surface/on_nurbs/fitting_surface_pdm.h>


class NurbsSurface;

typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > vector_vec2f;
typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > vector_vec3f;
struct NurbsDataSurface
{
	Eigen::Matrix3d eigenvectors;
	Eigen::Vector3d mean;

	Eigen::MatrixXd interior;
	std::vector<double> interior_weight;
	std::vector<double> interior_error;
	vector_vec2f interior_param;
	vector_vec3f interior_line_start;
	vector_vec3f interior_line_end;
	vector_vec3f interior_normals;

	vector_vec3f boundary;
	std::vector<double> boundary_weight;
	std::vector<double> boundary_error;
	vector_vec2f boundary_param;
	vector_vec3f boundary_line_start;
	vector_vec3f boundary_line_end;
	vector_vec3f boundary_normals;

	vector_vec3f common_boundary_point;
	std::vector<unsigned> common_boundary_idx;
	vector_vec2f common_boundary_param;

	std::vector<unsigned> common_idx;
	vector_vec2f common_param1;
	vector_vec2f common_param2;

	inline void
		clear_interior()
	{
		//interior.clear();
		interior = Eigen::Matrix3d::Identity();
		interior_weight.clear();
		interior_error.clear();
		interior_param.clear();
		interior_line_start.clear();
		interior_line_end.clear();
		interior_normals.clear();
	}

	inline void
		clear_boundary()
	{
		boundary.clear();
		boundary_weight.clear();
		boundary_error.clear();
		boundary_param.clear();
		boundary_line_start.clear();
		boundary_line_end.clear();
		boundary_normals.clear();
	}

	inline void
		clear_common()
	{
		common_idx.clear();
		common_param1.clear();
		common_param2.clear();
	}

	inline void
		clear_common_boundary()
	{
		common_boundary_point.clear();
		common_boundary_idx.clear();
		common_boundary_param.clear();
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


class NurbsSurface
{
public:
	void initNurbsPCA(int order, NurbsDataSurface* m_data, Eigen::Vector3d z= Eigen::Vector3d(0.0, 0.0, 1.0));

	Vector2d inverseMapping(const Vector3d& pt, const Vector2d& hint, double& error,
		Vector3d& p, Vector3d& tu, Vector3d& tv, int maxSteps, double accuracy, bool quiet=true);



	~NurbsSurface()
	{
		//cout << m_cv;
		delete m_cv;
		delete m_knot[0];
		delete m_knot[1];
	}

	NurbsSurface();
	// ¿½±´¹¹Ôìº¯Êý
	NurbsSurface(const NurbsSurface& other);
	NurbsSurface& operator=(const NurbsSurface& other);


	bool Create(int dim, bool is_rat, int order0, int order1, int cv_count0, int cv_count1);

	int KnotCount(int dir);

	bool ReserveKnotCapacity(int dir, int capacity);

	bool ReserveCVCapacity(int capacity);

	bool MakeClampedUniformKnotVector(int dir, double delta);

	bool ON_MakeClampedUniformKnotVector(int order, int cv_count, double* knot, double delta);

	bool ON_ClampKnotVector(int order, int cv_count, double* knot, int end);

	int ON_KnotCount(int order, int cv_count)
	{
		return (order + cv_count - 2);
	}


	bool Evaluate( // returns false if unable to evaluate
			double s, double t,       // evaluation parameter
			int der_count,  // number of derivatives (>=0)
			int v_stride,   // v[] array stride (>=Dimension())
			double* v,      // v[] array of length stride*(ndir+1)
			int side=0,       // optional - determines which side to evaluate from
							//         0 = default
							//         1 = from NE quadrant
							//         2 = from NW quadrant
							//         3 = from SW quadrant
							//         4 = from SE quadrant
			int hint[2] = nullptr       // optional - evaluation hint (int) used to speed
							//            repeated evaluations
		) const;


	std::vector < double> getElementVector(int dim);

	bool IsClosed(int dir) const;

	bool IsPeriodic(int dir) const;

	double Knot(int dir, int i) const;
	const double* Knot(int dir) const;

	bool InsertKnot(
		int dir,         // dir 0 = "s", 1 = "t"
		double knot_value,
		int knot_multiplicity=1 // default = 1
	);


	bool IsValid() const;
	int CVSize() const;
	int Order(int dir) const;
	bool SetCV(int i, int j, const PointT& point);
	double* CV(int i, int j) const;
	int CVCount(void) const;
	int CVCount(int dir) const;
	int Degree(int dir) const;
	ON_Interval Domain(int dir) const;

	Vector2d inverseMappingBoundary(const Vector3d& pt, double& error, Vector3d& p,
			Vector3d& tu, Vector3d& tv, int maxSteps=100, double accuracy=1e-6, bool quiet=true);

	Vector2d inverseMappingBoundary(const Vector3d& pt, int side, double hint,
			double& error, Vector3d& p, Vector3d& tu, Vector3d& tv, int maxSteps = 100, double accuracy = 1e-6, bool quiet = true);

	bool GetCV(int i, int j, PointT& point) const;

public:
	double* m_cv;
	int 	m_cv_capacity;
	int 	m_cv_count[2];
	int 	m_cv_stride[2];
	int 	m_dim;
	int 	m_is_rat;
	double* m_knot[2];
	int 	m_knot_capacity[2];
	int 	m_order[2];

};


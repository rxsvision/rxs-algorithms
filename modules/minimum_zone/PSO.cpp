#include "PSO.h"
#include<random>



float generateRandom01()
{
	std::random_device rd;
	std::default_random_engine generator(rd());
	std::uniform_real_distribution<double> distribution(0.0, 1.0);
	return distribution(generator);
}

PSO::PSO(CP cloud_in)
{
	cloud = cloud_in;
	n_particles = 30;
	best_fitness = 999;
	int max_degree = 5;
	for (int i = 0; i < n_particles; i++)
	{
		Particle p;
		p.pos = Vector3f::Random();
		p.pos.normalize();
		float deviation_angle_degrees = generateRandom01() * max_degree;
		float deviation_angle_radians = deviation_angle_degrees * M_PI / 180.0;
		Eigen::AngleAxisf rotation(deviation_angle_radians, p.pos.cross(Eigen::Vector3f::UnitZ()));
		p.pos = rotation * Eigen::Vector3f::UnitZ();
		
		p.speed = Vector3f::Random();
		p.speed /= (p.speed.norm() * 100);
		p.fitness = CalFitness(p);		
		p.best_pos = p.pos;
		p.best_fitness = p.fitness;
		particles.push_back(p);
	}
}

double PSO::CalFitness(Particle &p)
{
	auto norm = p.pos;
	norm.normalize();
	auto z_data = norm.transpose() * cloud->getMatrixXfMap(3, 4, 0);
	double flatness = z_data.maxCoeff() - z_data.minCoeff();

	p.history_fitness.push_back(flatness);

	if (flatness < 1e-10)
		return 99999;
	return flatness;
}

void PSO::move(Particle &p)
{
	p.pos += p.speed * p.learn_rate;
	//if (p.pos.squaredNorm() < 1e-10)
	//	p.pos = Vector3f(0, 0, 1);
	//p.pos.normalize();
	
	bool better = false;
	auto fitness = CalFitness(p);

	if (fitness > p.fitness)
		better = false;
	else
		better = true;


	p.fitness = fitness;

	if (fitness < p.best_fitness)
	{
		p.best_fitness = fitness;
		p.best_pos = p.pos;

		if (fitness < best_fitness)
		{
			best_fitness = fitness;
			best_pos = p.pos;
		}
	}
	
	Vector3f i_suppose = p.best_pos - p.pos;
	Vector3f we_suppose = best_pos - p.pos;
	//p.speed = p.learn_rate * 1 * i_suppose + p.learn_rate * 2 * we_suppose + 0.1 * p.speed;
	i_suppose.normalize();
	we_suppose.normalize();
	if(better)
		p.speed = 0.4 * i_suppose + 0.4 * we_suppose + 0.2 * p.speed;
	else
		p.speed = 0.5 * i_suppose + 0.2 * we_suppose + 0.3 * p.speed;
	p.speed.normalize();
	
	//if(better)
	//	p.learn_rate *= 1.2;
	//else
	//	p.learn_rate *=0.8;
	p.learn_rate *= 0.8;
	if (p.learn_rate < 0.00001) p.learn_rate = 0.00001;
}


void PSO::process()
{
	int iteration = 100;
	for (int _ = 0; _ < iteration; _++)
	{
		#pragma omp parallel for
		for (int i=0; i<particles.size(); i++)
		{
			move(particles[i]);
		}
	}

	//for (auto& p : particles)
	//{
	//	//drawLineGraph(p.history_fitness);
	//	for (auto& value : p.history_fitness)
	//		cout << value<<"  ";
	//	cout << endl;
	//}  
}
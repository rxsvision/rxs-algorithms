#pragma once
#include"czxTool.h"


struct Particle
{
	Vector3f pos;
	Vector3f speed;
	double fitness;
	Vector3f best_pos;
	double best_fitness;
	double learn_rate=0.1;

	vector<double> history_fitness;

};

class PSO
{
public:
	PSO(CP cloud_in);
	double CalFitness(Particle &p);
	void move(Particle &p);
	void process();
	double getBest() { return best_fitness; }
	Vector3f getBestPos() { return best_pos; }

private:	
	int n_particles;
	Vector3f best_pos;
	vector<Particle> particles;
	CP cloud;
	double best_fitness;
};


#pragma once
#include"czxTool.h"

struct Member
{
	int a, b, c;
	double fitness;
};

class GA
{
public:
	GA(CP cloud);
	
	void addMember(vector<Member> &population, Member m);

	double CalFitness(Member &m);
	void Mutate(Member &m);
	void Cross(Member &m);
	void process();

	double getBest() { return best_fitness; }

private:
	int population_size;
	int mutation_probability;
	CP cloud;
	double best_fitness;
	vector<Member> members;
	vector<Member> buffer;

	int iteration;
	int live;
	int strong;
};


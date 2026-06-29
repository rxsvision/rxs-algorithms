#include "GA.h"
#include <random>

int generateRandomNumber(int min, int max) 
{
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<> dis(min, max);
	return dis(gen);
}
template<typename T>
T randomSelection(T num1, T num2) {
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<> dis(0, 1);  // 生成0或1，代表选取第一个数或第二个数

	int randomNumber = dis(gen);

	return (randomNumber == 0) ? num1 : num2;
}
template<typename T, typename... RES>
void executeFunctionBasedOnProbability(double probability, T func, RES... param) 
{
	double randomValue = static_cast<double>(rand()) / RAND_MAX;

	if (randomValue < probability) {
		func(param...);
	}
}
double GA::CalFitness(Member &m)
{
	auto x = cloud->points[m.b].getVector3fMap() - cloud->points[m.a].getVector3fMap();
	auto y = cloud->points[m.c].getVector3fMap() - cloud->points[m.a].getVector3fMap();
	auto z = x.cross(y);
	z.normalize();
	auto z_data = z.transpose() * cloud->getMatrixXfMap(3, 4, 0);
	double flatness = z_data.maxCoeff() - z_data.minCoeff();
	if (flatness < 1e-10)
		return 99999;
	return flatness;
}
void GA::addMember(vector<Member> &population, Member m)
{
	best_fitness = min(best_fitness, m.fitness);
	if (m.fitness < live * best_fitness)
		population.push_back(m);
}
void GA::Mutate(Member &m)
{
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<> dis(1, 3);

	int randomNumber = dis(gen);
	if (randomNumber == 1) {
		m.a = generateRandomNumber(0, cloud->size()-1);
	}
	else if (randomNumber == 2) {
		m.b = generateRandomNumber(0, cloud->size()-1);
	}
	else {
		m.c = generateRandomNumber(0, cloud->size()-1);
	}
}
void GA::Cross(Member &m)
{
	Member wife = members[generateRandomNumber(0, members.size()-1)];
	Member children;
	children.a = randomSelection(m.a, wife.a);
	children.b = randomSelection(m.b, wife.b);
	children.c = randomSelection(m.c, wife.c);
	auto func = std::bind(&GA::Mutate, this, std::placeholders::_1);
	executeFunctionBasedOnProbability(mutation_probability, func, m);
	children.fitness = CalFitness(children);	
	addMember(buffer, children);
	if (m.fitness < strong * best_fitness)
		buffer.push_back(m);
}
GA::GA(CP cloud_in)
{
	this->cloud = cloud_in;
	population_size = cloud->size();
	mutation_probability = 0.1;
	best_fitness = 9999;
	live = 2;
	strong = 1.7;
	iteration = 10;
	for (int i = 0; i < population_size; i++)
	{
		Member m;
		m.a = i;
		m.b = generateRandomNumber(0, cloud->size()-1);
		m.c = generateRandomNumber(0, cloud->size()-1);
		m.fitness = CalFitness(m);
		addMember(members, m);
	}
}

void GA::process()
{
	for(int i = 0; i < iteration; i++)
	{
		//#pragma omp parallel for
		for (int j = 0; j<members.size();j++)
		{
			Cross(members[j]);
		}
		members = buffer;
		//cout << members.size() << endl;
		buffer.clear();
	}
}
/*
 * The XCS Library 
 * A C++ framework to apply and develop learning classifier systems
 * Copyright (C) 2002-2009 Pier Luca Lanzi and Daniele Loiacono
 * 
 * Pier Luca Lanzi and Daniele Loiacono
 * Dipartimento di Elettronica e Informazione
 * Politecnico di Milano
 * Piazza Leonardo da Vinci 32
 * I-20133 MILANO - ITALY
 * pierluca.lanzi@polimi.it - loiacono@elet.polimi.it
 *
 * This file is part of the XCSLIB library.
 *
 * xcslib is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * xcslib is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * A copy of the license is available at http://www.gnu.org/licenses/gpl.html
 * 
 * If you use this code, please cite the following technical report:
 *
 * P.L. Lanzi and D. Loiacono (2009), "XCSLib: The XCS Classifier System Library", 
 * Technical Report No. 2009005, Illinois Genetic Algorithms Laboratory
 * University of Illinois at Urbana-Champaign, 117 Transportation Building
 * 104 S. Mathews Avenue Urbana, IL 61801
 * 
 * Available at http://www.illigal.uiuc.edu/pub/papers/IlliGALs/2009005.pdf
 *
 * For updates please visit: http://xcslib.sf.net 
 *                           http://www.pierlucalanzi.net
 */




#include <cassert>
#include <string>
#include <fstream>
#include <algorithm>
#include "xcs_definitions.hpp"
#include "xcs_random.hpp"
#include "xcs_config_mgr2.hpp"
#include "real_interval_condition.hpp"


using namespace std;

#define __INTERVAL_TEXT_FORMAT__ "[%lf;%lf]"
#define __INTERVAL_GRAPH_OPEN_CHAR__ '|'
#define __INTERVAL_GRAPH_CLOSE_CHAR__ '|'
#define __INTERVAL_GRAPH_FULL_CHAR__ 'O'
#define __INTERVAL_GRAPH_PART_CHAR__ 'o'
#define __INTERVAL_GRAPH_NOT_CHAR__ '.'

#define __REAL_INTERVAL_CONDITION_SEPARATOR__ ','


bool			real_interval_condition::init = false;
unsigned long		real_interval_condition::dim;
double		        real_interval_condition::min_input;
double  	  	real_interval_condition::max_input;
double  		real_interval_condition::r0;
double  		real_interval_condition::m0;
t_mutation_type 	real_interval_condition::mutation_type;
unsigned long 		real_interval_condition::crossover_type;

real_interval_condition::real_interval_condition()
{
	if (!real_interval_condition::init)
	{
		xcs_utility::error(class_name(),"real_interval_condition()", "not inited", 1);
	}
	value.reserve(dim);
}

real_interval_condition::real_interval_condition(unsigned long d, double min_in, double max_in, double r, double m)
{
	init = true;
	dim = d;
	min_input =min_in;
	max_input = max_in;
	r0 = r;
	m0 = m;
	value.reserve(dim);
}

real_interval_condition::real_interval_condition(const real_interval_condition& cond)
{
	value.reserve(dim);
	for (int i=0; i<dim; i++)
	{
		xcslib::interval<double> interv(cond.value[i]);
		value[i] = interv;
	}
}

real_interval_condition::real_interval_condition(xcs_config_mgr2& xcs_config)
{
	string	str_mutation_type;
	string	str_crossover_type;

	if (!real_interval_condition::init)
	{
		if (!xcs_config.exist(tag_name()))
		{
			xcs_utility::error(class_name(), "constructor", "section <" + tag_name() + "> not found", 1);	
		}
		
		try {
			dim = xcs_config.Value(tag_name(), "input size");
			min_input = xcs_config.Value(tag_name(), "min input");
			max_input = xcs_config.Value(tag_name(), "max input");

			r0 = xcs_config.Value(tag_name(), "r0");
			m0 = xcs_config.Value(tag_name(), "m0");

			str_mutation_type = (string) xcs_config.Value(tag_name(), "mutation method");
			str_crossover_type = (string) xcs_config.Value(tag_name(), "crossover method");

		} catch (const char *attribute) {
			string msg = "attribute \'" + string(attribute) + "\' not found in <" + tag_name() + ">";
			xcs_utility::error(class_name(), "constructor", msg, 1);
		}

                set_mutation_type(str_mutation_type);
                set_crossover_type(str_crossover_type);
	}
	init = true;
}

bool
real_interval_condition::operator<(const real_interval_condition& cond) const
{
	return (string_value()<cond.string_value());
};

string
real_interval_condition::string_value()
const
{
	int sz = size();
	ostringstream out;
	for (int i=0; i< sz; i++)
	{
		if (i!=sz-1)
			out << value[i].string_value() << __REAL_INTERVAL_CONDITION_SEPARATOR__;
		else
			out << value[i].string_value();
	}
	return out.str();
}

void
real_interval_condition::set_string_value(string str)
{
	int pos, old_pos;
	int curr_dim=0;
	old_pos=0;
	pos = str.find(__REAL_INTERVAL_CONDITION_SEPARATOR__);
	while (pos != string::npos)
	{
		string sub_str(str,old_pos,(pos-old_pos));

		xcslib::interval<double> interv;
		interv.set_string_value(sub_str);
		assert(curr_dim < dim);
		value[curr_dim++] = interv;
		old_pos = pos;
		pos = str.find(__REAL_INTERVAL_CONDITION_SEPARATOR__,++old_pos);
	}
	
	if (pos==string::npos)
	{
		string sub_str(str,old_pos,(str.size()-old_pos));
		xcslib::interval<double> interv;
		interv.set_string_value(sub_str);
		assert(curr_dim < dim);
		value[curr_dim++] = interv;
	}
	assert (curr_dim == dim);
}

bool
real_interval_condition::operator==(const real_interval_condition& cond)
const
{
	for (int i=0; i <dim; i++)
	{
		if (value[i]!=cond.value[i])
			return false;
	}
	return true;
};

bool
real_interval_condition::operator!=(const real_interval_condition& cond)
const
{
	for (int i=0; i <dim; i++)
	{
		if (value[i]!=cond.value[i])
			return true;
	}
	return false;
}

real_interval_condition&
real_interval_condition::operator=(real_interval_condition& cond)
{
	value.clear();
        value.reserve(dim);
	for (int i=0; i<dim; i++)
	{
		xcslib::interval<double> interv(cond.value[i]);
		value[i]=interv;
	}
}

real_interval_condition&
real_interval_condition::operator=(const real_interval_condition& cond)
{
	value.clear();
	value.reserve(dim);
	for (int i=0; i<dim; i++)
	{
		xcslib::interval<double> interv(cond.value[i]);
		value[i]=interv;
	}
	return (*this);
}

void
real_interval_condition::normalize(vector<double>& input, vector<double>& norm_input)
{
	assert (input.size()==dim);

	norm_input.clear();

        for (int i=0; i<dim; i++)
        {
		double delta = value[i].get_upper_bound() - value[i].get_lower_bound();
		double curr;
		if (delta!=0)
			curr = (input[i] - value[i].get_lower_bound())/delta;
		else
			curr = 1;
		norm_input.push_back(curr);
        }
}

bool
real_interval_condition::match(const real_inputs& sens) const
{
	bool result = true;
	int sz = size();

	if (sens.size()!=sz)
		cout << "sens.size()=" << sens.size() << " - sz=" << sz << endl;
	assert(sens.size()== sz);

	for (int i=0; i<dim; i++) {
		double curr = sens.input(i);
		if ((curr < value[i].get_lower_bound())||(curr > value[i].get_upper_bound()))
		{
			result = false;
			break;
		}
	}
	return result;
}

void
real_interval_condition::cover(const real_inputs& sens)
{
	assert (sens.size()==dim);
        value.clear();
	value.reserve(dim);
	for (int i = 0; i< dim; i++)
	{
                double tmp;
		xcslib::interval<double> tmp_int;
		tmp = sens.input(i) - r0*xcs_random::random();
		if (tmp < min_input)
			tmp = min_input;
		tmp_int.set_lower_bound(tmp);
		tmp = sens.input(i) + r0*xcs_random::random();
		if (tmp > max_input)
			tmp = max_input;
		tmp_int.set_upper_bound(tmp);
		value[i] = tmp_int;
	}
}


void 
real_interval_condition::fixed_mutation(double mu)
{
	for (int i=0; i<dim; i++)
	{
		if (xcs_random::random () < mu)
		{
			double old_lower = value[i].get_lower_bound();
			double new_lower = old_lower + m0*(xcs_random::random()) * xcs_random::sign();
			value[i].set_lower_bound(new_lower);
		}

		if (xcs_random::random () < mu)
		{
			double old_upper = value[i].get_upper_bound();
			double new_upper = old_upper + m0*(xcs_random::random()) * xcs_random::sign();
			value[i].set_upper_bound(new_upper);
		}
		check(value[i]);
	}
}

// void 
// real_interval_condition::wilson_restricted_mutate(const real_inputs &sens, const double& mu)
// {
// 	assert (sens.size() == dim);
// 
// 	for (int i=0; i<dim; i++)
// 	{
// 		double curr_input = sens.input(i);
// 
// 		if (xcs_random::random () < mu)
// 		{
// 			double old_lower = value[i].get_lower_bound();
// 			double new_lower = old_lower + m0*(xcs_random::random()) * xcs_random::sign();
// 
// 			if (new_lower > curr_input)
// 				new_lower = curr_input;
// 
// 			value[i].set_lower_bound(new_lower);
// 		}
// 
// 		if (xcs_random::random () < mu)
// 		{
// 			double old_upper = value[i].get_upper_bound();
// 			double new_upper = old_upper + m0*(xcs_random::random()) * xcs_random::sign();
// 
// 			if (new_upper < curr_input)
// 				new_upper = curr_input;
// 
// 			value[i].set_upper_bound(new_upper);
// 		}
// 		check(value[i]);
// 	}
// }

void 
real_interval_condition::proportional_mutation(double mu)
{
	for (int i=0; i<dim; i++)
	{
		if (xcs_random::random () < mu)
		{
			
			//! Butz's mutation ...
			//help <- urand() * (x2-x1)
			double x1,x2;
			
			double range = (value[i].get_upper_bound()-value[i].get_lower_bound());
			
			double center =  xcs_random::random()*range+value[i].get_lower_bound();
				
			//x1 <- help - .5 * (.5+.5*urand()) * (x2-x1)
			//x2 <- help + help - x1 //x1 is the new value of x1
			//-> help is the new middle where urand() is unif. rand. number between 0 and 1
			//now the new x1 and x2:
			//with prob .5 interval size is decreased:
			
			if (xcs_random::random()<.5)
			{	//! decrease
				double sigma = (.5+.5*xcs_random::random()) * range;
				x1 = center - .5 * sigma;
				x2 = center+center-x1;
				//center + .5 * sigma; //x1 is the new value of x1
			} else {//! increase
				double sigma = (1+.5*xcs_random::random()) * range;
				x1 = center - .5 * sigma;
				x2 = center+center-x1;
				//x2 = center + .5 * sigma; //x1 is the new value of x1	
			}
				
			value[i].set_lower_bound(x1);
			value[i].set_upper_bound(x2);
			check(value[i]);
		}
	}
}

void
real_interval_condition::mutate(const real_inputs &sens, const double& mu)
{
	assert (sens.size() == dim);

	mutate(mu);
}

void
real_interval_condition::mutate(const double& mu)
{
	switch (mutation_type)
	{
		case MUTATION_FIXED:
			fixed_mutation(mu);
			break;
		case MUTATION_PROPORTIONAL:
			proportional_mutation(mu);
			break;
		default:
			xcs_utility::error(class_name(),"mutate","unrecognized mutation type",1);
	}
}

void
real_interval_condition::recombine(real_interval_condition& offspring, unsigned long method)
{
	switch (crossover_type)
	{
		case 0:
			uniform_crossover(offspring);
			break;
		case 1:
			single_point_crossover(offspring);
			break;
		case 2:
			two_points_crossover(offspring);
			break;
		default:
			xcs_utility::error(class_name(),"mutate","unrecognized mutation type",1);
	}
}

bool
real_interval_condition::is_more_general_than(const real_interval_condition& cond)
const
{
	unsigned int sz = size();
	assert (cond.size() == sz);
	for(int i=0; i<dim; i++)
	{
		if (!(cond.value[i]<value[i]))
			return false;
	}
	return true;
}

void
real_interval_condition::random()
{
	//cout << "******** real_interval_condition: random()  ***********" << endl;
        value.clear();
	value.reserve(dim);
	for (int i=0; i<dim; i++)
	{
		double int_size = (max_input-min_input-1) * xcs_random::random();
		double lower_bound = min_input + (max_input-min_input-int_size)*xcs_random::random();
		xcslib::interval<double> curr(lower_bound, lower_bound+int_size);
		value[i] = curr;
	}
}

//! uniform crossover
void
real_interval_condition::uniform_crossover(real_interval_condition& offspring)
{
 	unsigned long sz = size();
	
 	for(unsigned long it=0; it<sz; it++)
 	{
 		if (xcs_random::random()<.5)
		{
			//! swap lower
			value[it].swap_lower(offspring.value[it]);
		}
		
		if (xcs_random::random()<.5)
		{
			//! swap upper
			value[it].swap_upper(offspring.value[it]);
		}
		
		check(value[it]);
		check(offspring.value[it]);	
 	}
}


//! single point crossover
void
real_interval_condition::single_point_crossover(real_interval_condition& offspring)
{
	int sz = size();
	int pos;

	pos = xcs_random::dice(sz);

	if (xcs_random::random()<.5)
	{
		value[pos].swap(offspring.value[pos]);
	}
	else
	{
		value[pos].swap_upper(offspring.value[pos]);
		check(value[pos]);
		check(offspring.value[pos]);
	}
		
	for (int i=pos+1; i<dim; i++)
	{
		value[i].swap(offspring.value[i]);
	}
}

//! double point crossover
void
real_interval_condition::two_points_crossover(real_interval_condition& offspring)
{
	int sz = size();
	
	int pos1 = xcs_random::dice(sz);
	int pos2 = xcs_random::dice(sz);

	if (pos1>pos2)
	{
		int tmp;
		tmp = pos1;
		pos1 = pos2;
		pos2 = tmp;
	}


	if (xcs_random::random()<.5)
	{
		value[pos1].swap(offspring.value[pos1]);
	}
	else
	{
		value[pos1].swap_upper(offspring.value[pos1]);
		check(value[pos1]);
		check(offspring.value[pos1]);
	}

	for (int i=pos1+1; i<pos2; i++)
	{
		value[i].swap(offspring.value[i]);
	}

	if (xcs_random::random()<.5)
	{
		value[pos2].swap(offspring.value[pos2]);
	}
	else
	{
		value[pos2].swap_lower(offspring.value[pos2]);
		check(value[pos2]);
		check(offspring.value[pos2]);
	}
}

void 
real_interval_condition::check(xcslib::interval<double>& it)
{
	//! sort the lower and upper boundaries
	double lb = it.get_lower_bound();
	double ub = it.get_upper_bound();
	
	//! if bound are unsorted
	if (lb>ub)
	{
		it.set_lower_bound(ub);	
		it.set_upper_bound(lb);	
	}
	
	if (it.get_lower_bound()<real_interval_condition::min_input)
	{
		it.set_lower_bound(real_interval_condition::min_input);
	}
	
	if (it.get_upper_bound()>real_interval_condition::max_input)
	{
		it.set_upper_bound(real_interval_condition::max_input);
	}
}

void 
real_interval_condition::set_crossover_type(string str)
const
{
	if (str.compare("onepoint")==0)
		crossover_type = CROSSOVER_ONEPOINT;
	else if (str.compare("twopoints")==0)
		crossover_type = CROSSOVER_TWOPOINTS;
	else if (str.compare("uniform")==0)
		crossover_type = CROSSOVER_UNIFORM;
	else
		xcs_utility::error("real_interval_condition","set_crossover_type","unrecognized crossover type",1);
};

void 
real_interval_condition::set_mutation_type(string str) const
{
	if (str.compare("fixed")==0)
		mutation_type = MUTATION_FIXED;
	else if (str.compare("proportional")==0)
		mutation_type = MUTATION_PROPORTIONAL;
	else
		xcs_utility::error(class_name(),"set_mutation_type","unrecognized mutation type",1);
};

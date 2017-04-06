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




//-------------------------------------------------------------------------
// Filename      : real_functions_env.cpp
//
// Purpose       : implement several real functions
//
// Special Notes :
//
//
// Creator       : Pier Luca Lanzi and Daniele Loiacono
//
// Creation Date :
//
// Current Owner :
//-------------------------------------------------------------------------

/*!
 * \file  real_functions_env.cpp
 *
 * \brief implement several real functions
 *
 */

#include "real_functions_env.hpp"

bool		real_functions_env::init = false;
double		real_functions_env::min_input;
double		real_functions_env::max_input;
unsigned long	real_functions_env::no_inputs;

void 
real_functions_env::init_function(string str_fun, unsigned long &no_inputs)
{
	if (str_fun=="SINE") 	{ function = &real_functions_env::rf_sin;	no_inputs = 1;}
	if (str_fun=="SINE3")	{ function = &real_functions_env::rf_sin3; 	no_inputs = 1;}
	if (str_fun=="SINE4")	{ function = &real_functions_env::rf_sin4; 	no_inputs = 1;}
}

real_functions_env::real_functions_env(xcs_config_mgr2& xcs_config)
{
	string str_mutation_type;
	string str_crossover_type;
	string str_selected_function;

	if (!real_functions_env::init)
	{
		if (!xcs_config.exist(tag_name()))
		{
			xcs_utility::error(class_name(), "constructor", "section <" + tag_name() + "> not found", 1);	
		}
		
		try {
			//! input range
			min_input = xcs_config.Value(tag_name(), "min input", 0.0);
			max_input = xcs_config.Value(tag_name(), "man input", 1.0);

			//! function to be approximated
			str_selected_function = (string) xcs_config.Value(tag_name(), "function");

			//! scale factor that multiplies both input and output (it zooms)
			sf = xcs_config.Value(tag_name(), "scale factor", 1.0);

			/*! 
			 * determine the resolution used to test the final result
			 */ 
			sampling_resolution = xcs_config.Value(tag_name(), "sampling_resolution", 0.01);

		} catch (const char *attribute) {
			string msg = "attribute \'" + string(attribute) + "\' not found in <" + tag_name() + ">";
			xcs_utility::error(class_name(), "constructor", msg, 1);
		}

		init_function(str_selected_function, no_inputs);

		problem_in.reserve(no_inputs);

#ifdef __RF_DEBUG__
		//! debug section
		cout << "Function selected: " << str_selected_function << endl;
		cout << "# of function selected: " << selected_function << endl;
		cout << "# inputs: " << no_inputs << endl;
#endif

		current_inputs.clear();

		for(int i=0;i<no_inputs;i++)
		{
			current_inputs.push_back(min_input);
		}

/*		if (string(str_input_type)=="integer")
			flag_real_inputs = false;
		else if (string(str_input_type)=="real")
			flag_real_inputs = true;
		else xcs_utility::error(class_name(),"class constructor", string("range type <")+string(str_input_type)+"> not recognized", 1);*/
	}	
	real_functions_env::init = true;
}






void
real_functions_env::begin_problem(const bool explore)
{
	for (int i=0;i<no_inputs;i++)
	{
		current_inputs[i] = xcs_random::random()*(max_input - min_input) + min_input;
	}

	real_inputs tmp(no_inputs);

	for(int i=0;i<no_inputs;i++)
		tmp.set_input(i,current_inputs[i]);

	inputs = tmp;
}

void
real_functions_env::trace(ostream& output) const
{
	int old_precision = output.precision();
	output.setf(ios::scientific);
	output.precision(5);
	output << current_reward;
	output.unsetf(ios::scientific);
	output.precision(old_precision);
}

void
real_functions_env::save_state(ostream& output) const
{
/*	output << endl;
	for(int i=0;i<no_inputs;i++)
	{
		output << current_inputs[i] << endl;
	}*/
}

void
real_functions_env::restore_state(istream& input)
{
#ifdef __DEBUG__
	cout << "Begin restore_state... " << endl;
#endif
/*	real_inputs tmp(no_inputs);
	
	for(int i=0;i<no_inputs;i++)
	{
		input >> x;
		tmp.set_input(i,x);
	}

	inputs = tmp;*/
}

void
real_functions_env::reset_problem()
{
	real_inputs tmp(no_inputs);

	for (int i=0;i<no_inputs;i++)
	{
		problem_in[i] = min_input;
	}

	for (int i=0;i<no_inputs;i++)
	{
		tmp.set_input(i,min_input);
	}

	inputs = tmp;
}

bool
real_functions_env::next_problem()
{
	real_inputs tmp(no_inputs);
	double step_size; 
	
	step_size = sampling_resolution;
	
	bool stop=false;
	problem_in[no_inputs-1]+=step_size;


	for(int i=0;i<=no_inputs;i++)
	{
		if (stop == false)
		{
			if (problem_in[no_inputs-1-i]>max_input)
			{
				if (no_inputs-1-i == 0)
				{
					return false;
				}
				problem_in[no_inputs-1-i]=min_input;
				problem_in[no_inputs-1-i-1]+=step_size;
			}
			else
			{
				stop = true;
			}
		} else {
			for(int i=0;i<no_inputs;i++)
			{
				current_inputs[i]=problem_in[i];
				tmp.set_input(i,problem_in[i]);
				inputs = tmp;
			}
			return true;
		}
	}
}

void
real_functions_env::reset_input()
{
	reset_problem();
}

bool
real_functions_env::next_input()
{
	return next_problem();
}

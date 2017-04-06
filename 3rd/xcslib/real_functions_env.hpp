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
// Filename      : real_functions_env.hpp
//
// Purpose       : real functions
//
// Special Notes :
//
// Creator       :
//
// Creation Date :
//
// Modifications :
//
// Current Owner :
//-------------------------------------------------------------------------

/*!
 * \file real_functions_env.hpp
 *
 * \brief implements various environments to test XCSF 
 *
 */

// How to include a new function.
// In real_function.hh:
// 1) Add FUNTION_XXX in typedef enum
// 2) Add the function output in compute_function
// In real_functions.cc:
// 1) Add label identification and number of inputs setting in
//    real_functions_env::init_function

#include "rl_definitions.hpp"
#ifndef __REAL_FUNCTIONS_ENV__
#define __REAL_FUNCTIONS_ENV__

#include <sstream>
#include <cmath>

#include "environment_base.hpp"
#include "xcs_config_mgr2.hpp"

/*!
 * \class functions_env functions_env.hpp
 * \brief implements various real functions
 * \sa environment_base
 */

// double	rf_sin(vector<double>& x) { assert(x.size()==1); return (sf)*std::sin((2*M_PI*x[0])/sf);};

class real_functions_env : public virtual environment_base
{
private:
	static bool		init;			//!< true if the class has been inited through the configuration manager
	static double 		min_input;		//! maximum possible input
	static double 		max_input;		//! minimum possible input
	static unsigned long	no_inputs;		//! number of inputs

	t_state			inputs;			//!< current input configuration
	vector<double>		current_inputs;		//!< same information as in "inputs" as a vector of double

	//! current reward returned for the last action performed
	double		current_reward;
	double		sf;				//!< scale factor for the target function
	
	double		sampling_resolution;		//!< specify the resolution used to test the learned function
/*	double		problem_x;
	double		problem_y;*/
// 	double		x,y;
	vector<double>	problem_in;

	//! functions implementations
	double	rf_sin(vector<double>& x) { assert(x.size()==1); return sf*std::sin((2*M_PI*current_inputs[0])/sf);};

	double	rf_sin3(vector<double>& x) {
		assert(x.size()==1); 
		return 	sf*(std::sin((2*M_PI*current_inputs[0])/sf)+std::sin((4*M_PI*current_inputs[0])/sf)+
			std::sin((6*M_PI*current_inputs[0])/sf));
	}

	double	rf_sin4(vector<double>& x) {
		assert(x.size()==1); 
		return 	sf*(std::sin((2*M_PI*current_inputs[0])/sf) + std::sin((4*M_PI*current_inputs[0])/sf) + \
			std::sin((6*M_PI*current_inputs[0])/sf) + std::sin((8*M_PI*current_inputs[0])/sf) );
	}

	double	rf_abs(vector<double>& x) {
		assert(x.size()==1); 
		return sf*std::fabs(std::sin((2*M_PI*current_inputs[0])/sf) + std::fabs(std::cos((2*M_PI*current_inputs[0])/sf)));
	}

	double	rf_abs2(vector<double>& x) {
		assert(x.size()==1); 
		return sf*std::fabs(std::sin((2*M_PI*current_inputs[0])/sf) + std::fabs(std::cos((2*2*M_PI*current_inputs[0])/sf)));
	}

	double	rf_pol(vector<double>& x) {
		assert(x.size()==1); 
		return sf*(1+(current_inputs[0]/sf)+std::pow((current_inputs[0]/sf),2)+std::pow((current_inputs[0]/sf),3));
	}

	unsigned long	arity;

	double		(real_functions_env::*function)(vector<double>& x);			//! selected function

public:
	string class_name() const { return string("real_functions_env"); };

	string tag_name() const { return string("environment::real_functions"); };

	//! Constructor for the sine environment class. It reads the class parameters through the configuration manager.
	/*!
	 *  This is the only constructor that can be used.
	 */
	real_functions_env(xcs_config_mgr2&);

	//! Destructor for the sine environment class.
	~real_functions_env() {};

	void begin_problem(const bool explore);

	void end_problem() {};

	void begin_experiment() {};

	void end_experiment() {};

	bool stop() const {return true;};

	void perform(const t_action& action)
        {
		current_reward = (*this.*function)(current_inputs);
        }

	void trace(ostream& output) const;

	bool allow_test() const {return true;};

	void reset_problem();
	bool next_problem();

	void reset_input();
	bool next_input();

	void save_state(ostream& output) const;
	void restore_state(istream& input);

	//! indicates that sine environments are single step problems
	virtual bool single_step() const {return true;};

	virtual double reward() const {assert(current_reward==real_functions_env::current_reward); return current_reward;};

	virtual t_state state() const {return inputs;};

	virtual void set_state(t_state sensor) {
		this->inputs = sensor;
		current_inputs.clear();
		int i;
		for(i=0;i<current_inputs.size();i++)
			current_inputs.push_back(sensor.input(i));
	};

	virtual void print(ostream& output) const { output << inputs.string_value();};

	void init_function(string str_fun, unsigned long &no_inputs);
};
#endif

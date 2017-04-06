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
// Filename      : real_interval_condition.hh
//
// Purpose       : implements real-interval conditions
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
 * \file real_interval_condition.hpp
 *
 * \author
 *
 * \version
 *
 * \date
 *
 * \brief class for real-interval conditions
 *
 */

/*!
 * \class real_interval_condition real_interval_condition.hh
 *
 * \brief class for real-interval conditions
 */

#include <string>
#include <sstream>
#include "rl_definitions.hpp"
#include "xcs_config_mgr2.hpp"
#include "real_inputs.hpp"
#include "interval.hpp"
#include "condition_base.hpp"

#ifndef __REAL_INTERVAL_CONDITION__
#define __REAL_INTERVAL_CONDITION__


#define __CONDITION_VERSION__ "real interval condition"

typedef enum
{
	MUTATION_PROPORTIONAL,
	MUTATION_FIXED,
} t_mutation_type;

typedef enum
{
	CROSSOVER_ONEPOINT,
	CROSSOVER_TWOPOINTS,
	CROSSOVER_UNIFORM
} t_crossover_type;

using namespace xcslib;

class real_interval_condition : public virtual condition_base<real_interval_condition, real_inputs>
{
private:

	vector< interval< double > > value;

	static bool		init;			//!< true if the class has been already inited through the configuration manager
	static unsigned long 	dim;
	static double 		min_input;
	static double 		max_input;
	static double 		r0;
	static double 		m0;

   	static t_mutation_type	mutation_type;
	static unsigned long 	crossover_type;

public:
	//! name of the class that implements the condition
	string class_name() const { return string("real_interval_condition"); };

	//! tag used to access the configuration file
	string tag_name() const { return string("condition::real_interval"); };

	//! return the condition as a string
	string string_value() const;

	//! set the condition to a value represented as a string
	void set_string_value(string str);

	//! return the condition size
	unsigned long size() const {return dim;};

	//
	//	class constructors
	//

	//! Constructor for the interval condition class that read the class parameters through the configuration manager
	/*!
	 *  This is the first constructor that must be used. Otherwise an error is returned.
	 */
	real_interval_condition(xcs_config_mgr2&);

	//! Default constructor for the interval condition class
	/*!
	 *  If the class parameters have not yet initialized through the configuration manager,
	 *  an error is returned and the method exists to shell.
	 *  \sa real_interval_condition(xcs_config_mgr&)
	 *  \sa xcs_config_mgr
	 */

	real_interval_condition();

	real_interval_condition(const real_interval_condition& cond);

	real_interval_condition(unsigned long d, double min_in, double max_in, double r=10, double  m=20);

	//
	//	class destructor
	//

	//! Default destructor for the interval condition class
	~real_interval_condition() { value.clear(); };

	//
	//	comparison operators
	//

	//! less than operator
	bool operator< (const real_interval_condition& cond) const;

	//! equality operator
	bool operator==(const real_interval_condition& cond) const;

	//! inequality operator
	bool operator!=(const real_interval_condition& cond) const;

	//
	//	assignment operators
	//

	//! assignment operator
	real_interval_condition& operator=(real_interval_condition& cond);

	//! assignment operator for a constant value
	real_interval_condition& operator=(const real_interval_condition& cond);

        // normalizza l'input
        void normalize(vector<double>& input, vector<double>&);
      
	//
	// 	match operator
	//

	//! return true if the condition matches the input configuration
	bool match(const real_inputs& input) const;

	//
	//	cover operator
	//

	//! set the condition to cover the input
	void cover(const real_inputs& input);

	//
	//	genetic operators
	//

	//! mutate the condition according to the mutation rate \emph mu
	void mutate(const double&);

	//! mutate the condition according to the mutation rate \emph mu; the mutating bits are set according to the current input
	void mutate(const real_inputs &sens, const double& mu);

	//! recombine the condition according to the strategy specified by the method variable
	// \param method the crossover type to be used
	void recombine(real_interval_condition&, unsigned long method=0);

	//
	//    pretty print the condition
	//

	//! pretty print the condition to the output stream "output".
	void print(ostream& output) const { output << string_value(); };


	//! true if the representation allow the use of GA subsumption
	virtual bool allow_ga_subsumption() {return true;};

	//! true if the representation allow the use of action set subsumption
	virtual bool allow_as_subsumption() {return true;};

	//! return true if the condition is more general than (i.e., subsumes) \emph cond
	bool is_more_general_than(const real_interval_condition& cond) const;
	
	double generality() const 
	{
		double delta = max_input-min_input+1;
		double gen = 0;
		int sz=0;
		for (int i=0; i< value.size(); i++)
		{
			gen += (value[i].get_upper_bound()-value[i].get_lower_bound() + 1);
			sz++;
		}
		return (gen/(sz*delta));
	};

	//! generate a random condition
	void random();
	
	//! additional functions 
	double	min() const {return min_input;};
	double	max() const {return max_input;};
	double	lower(long i) const {assert(i<size()); return value[i].get_lower_bound();};
	double	upper(long i) const {assert(i<size()); return value[i].get_upper_bound();};
	long	condition_size() const {return value.size();};
		
	void single_point_crossover(real_interval_condition& offspring);
	void two_points_crossover(real_interval_condition& offspring);
	void uniform_crossover(real_interval_condition& offspring);

	void fixed_mutation(double mu);

	void proportional_mutation(double mu);

	void set_crossover_type(string str) const;

	void set_mutation_type(string str) const;

	void check( xcslib::interval<double> &it);
};

#endif

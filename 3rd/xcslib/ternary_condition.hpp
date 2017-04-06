/*
 * The XCS Library 
 * A C++ framework to apply and develop learning classifier systems
 * Copyright (C) 2002-2009 Pier Luca Lanzi
 * 
 * Pier Luca Lanzi 
 * Dipartimento di Elettronica e Informazione
 * Politecnico di Milano
 * Piazza Leonardo da Vinci 32
 * I-20133 MILANO - ITALY
 * pierluca.lanzi@polimi.it/lanzi@elet.polimi.it
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
// Filename      : ternary_condition.hpp
//
// Purpose       : definition class for classifier ternary conditions
//                 
// Special Notes : 
//                 
//
// Creator       : Pier Luca Lanzi
//
// Creation Date : 2002/05/31
//
// Current Owner : Pier Luca Lanzi
//-------------------------------------------------------------------------

/*!
 * \file ternary_condition.hpp
 *
 * \author Pier Luca Lanzi
 *
 * \version 0.01
 *
 * \date 2002/06/10
 * 
 * \brief definition class for classifier ternary conditions
 *
 */

/*!
 * \class ternary_condition ternary_condition.h
 *
 * \brief definition class for classifier ternary conditions
 */

#ifndef __TERNARY_CONDITION__
#define __TERNARY_CONDITION__

#define __CONDITION_VERSION__ "ternary strings {0,1,#}"

#include <string>
#include "rl_definitions.hpp"
#include "xcs_config_mgr2.hpp"
#include "condition_base.hpp"
#include "binary_inputs.hpp"

class ternary_condition : public virtual condition_base<ternary_condition, binary_inputs> 
{
public:
	static const char	dont_care;			//! don't care symbol used in conditions

private:
	static bool		init;				//!< true if the class has been already inited through the configuration manager

	string			bitstring;						//!< condition string
	static unsigned long	no_bits;				//!< number of bits in condition
	static double		dont_care_prob;				//!< probability of having a don't care in a random condition
	static bool		flag_mutation_with_dontcare;	//!< true if #s are used in mutation
	static unsigned long	crossover_type;			//!< default crossover type (0=uniform, 1=one-point, 2=two-points)
	static unsigned long	mutation_type;			//!< default mutation type (1=niche mutation, 2=two-values, 3=pure or three values)

public:
	//! name of the class that implements the condition
	string class_name() const { return string("ternary_condition"); };
	
	//! tag used to access the configuration file
	string tag_name() const { return string("condition::ternary"); };

	//! return the condition as a string
	string string_value() const {return bitstring;};

	//! set the condition to a value represented as a string
	void set_string_value(string str) {bitstring = str;};

	//! return the condition size
	unsigned long size() const {return bitstring.size();};

	//
	//	class constructors
	//
	
	//! Constructor for the ternary condition class that read the class parameters through the configuration manager
	/*!
	 *  This is the first constructor that must be used. Otherwise an error is returned.
	 */
	ternary_condition(xcs_config_mgr2&);

	//! Default constructor for the ternary condition class
	/*!
	 *  If the class parameters have not yet initialized through the configuration manager, 
	 *  an error is returned and the method exists to shell. 
	 *  \sa ternary_condition(xcs_config_mgr&)
	 *  \sa xcs_config_mgr
	 */
	ternary_condition();

	//
	//	class destructor
	//
	
	//! Default destructor for the ternary condition class
	~ternary_condition();
	
	//	
	//	comparison operators
	//	
	
	//! less than operator
	bool operator< (const ternary_condition& cond) const;

	//! equality operator
	bool operator==(const ternary_condition& cond) const;

	//! inequality operator
	bool operator!=(const ternary_condition& cond) const;

	//
	//	assignment operators
	//
	
	//! assignment operator
	ternary_condition& operator=(ternary_condition& cond); 

	//! assignment operator for a constant value
	ternary_condition& operator=(const ternary_condition& cond);

	// 
	// 	match operator
	//
	
	//! return true if the condition matches the input configuration
	bool match(const binary_inputs& input) const;

	// 
	//	cover operator
	//
	
	//! set the condition to cover the input 
	void cover(const binary_inputs& input);

	//
	//	genetic operators
	//

	//! mutate the condition according to the mutation rate \emph mu
	void mutate(const double&);

	//! mutate the condition according to the mutation rate \emph mu; the mutating bits are set according to the current input
	void mutate(const binary_inputs &sens, const double& mu);

	//! recombine the condition according to the strategy specified by the method variable
	// \param method the crossover type to be used
	void recombine(ternary_condition&, unsigned long method=crossover_type);

	// 
	//    pretty print the condition
	//   
	
	//! pretty print the condition to the output stream "output".
	void print(ostream& output) const { output << bitstring; };


	//! true if the representation allow the use of GA subsumption
	virtual bool allow_ga_subsumption() {return true;};

	//! true if the representation allow the use of action set subsumption
	virtual bool allow_as_subsumption() {return true;};

	//! return true if the condition is more general than (i.e., subsumes) \emph cond
	bool is_more_general_than(const ternary_condition& cond) const;

	//! generate a random condition
	void random();

 private:
	/// single point crossover
	void	single_point_crossover(ternary_condition&);
 
	/// two point crossover
	void	two_point_crossover(ternary_condition&);
 
	/// uniform crossover
	void	uniform_crossover(ternary_condition&);

 public:
	//! generality
	double generality() const;

	//! specificity
	double specificity() const;
};
#endif

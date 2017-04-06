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
// Filename      : condition_base.hpp
//
// Purpose       : template class for classifier conditions
//                 
// Special Notes : 
//                 
//
// Creator       : Pier Luca Lanzi
//
// Creation Date : 2002/05/31
//
// Current Owner : Pier Luca Lanzi
//
// Updates
//         2006 10 31 added specificity and generality
//-------------------------------------------------------------------------

/*!
 * \file condition_base.hpp
 *
 * \author Pier Luca Lanzi
 *
 * \version 0.01
 *
 * \date 2002/05/31
 * 
 * \brief template class for classifier conditions
 *
 */

/*!
 * \class condition_base condition_base.hh
 *
 * \brief template class for classifier conditions
 */

#ifndef __XCS_CONDITION__
#define __XCS_CONDITION__

#include <string>
#include "xcs_definitions.hpp"


template <class condition, class sensors>
class condition_base
{
public:
	/*!
	 * \fn string class_name() const
	 * \brief name of the class that implements the environment
	 * 
	 * This function returns the name of the class. 
	 * Every class must implement this method that is used to 
	 * trace errors.
	 */
	//! name of the class that implements the condition
	virtual string class_name() const = 0;
	
	//! tag used to access the configuration file
	virtual string tag_name() const = 0;

	//! return the condition as a string
	virtual string string_value() const = 0;

	//! set the condition to a value represented as a string
	virtual void set_string_value(string str) = 0;

	//! return the condition size
	virtual unsigned long size() const = 0;

	//! true if the representation allow the use of GA subsumption.
	virtual bool allow_ga_subsumption() {return false;};
	//! true if the representation allow the use of action set subsumption.
	virtual bool allow_as_subsumption() {return false;};
	
	//! less than operator
	virtual bool operator< (const condition& cond) const = 0;
	//! equality operator
	virtual bool operator==(const condition& cond) const = 0;
	//! inequality operator
	virtual bool operator!=(const condition& cond) const = 0;

	//! assignment operator
	virtual condition& operator=(condition& cond) = 0; 

	//! assignment operator for a constant value
	virtual condition& operator=(const condition& cond) = 0;

	//! returns true if the condition matches the input configuration
	virtual bool match(const sensors& input) const = 0;

	//! set the condition to cover the input 
	virtual void cover(const sensors& input) = 0;

	//! mutate the condition according to the mutation rate \emph mu
	virtual void mutate(const double& mu) = 0;

	//! mutate the condition according to the mutation rate \emph mu; the mutating bits
	virtual void mutate(const sensors& input, const double& mu) = 0;

	//! recombine the condition according to the strategy specified with the method variable
	virtual void recombine(condition&, unsigned long method=0) = 0;
	
	//! pretty print the condition to the output stream "output".
	virtual void print(ostream& output) const = 0;

	//! write the condition to an output stream 
	friend ostream& operator<<(ostream& output, const condition& cond)
	{
		output << cond.string_value();
		return (output);
	};

	//! read the condition from an input stream 
	friend istream& operator>>(istream& input, condition& cond)
	{
		string	str;
		input >> str;
		cond.set_string_value(str);
		return (input);
	};

	//! return true if the condition is subsumed by \emph cond
	virtual bool subsumed_by(const condition& cond) const {return false;};

	//! return true if the condition is more general than \emph cond
	virtual bool is_more_general_than(const condition& cond) const {return false;};

	//! generate a random condition
	virtual void random() = 0;

	//! virtual destructor
	virtual ~condition_base() {};
	
	//! version
	virtual string version() const {return "no information available";};

 public:
	//! generality
	double generality() const {assert(false);};

	//! specificity
	double specificity() const {assert(false);};
};
#endif

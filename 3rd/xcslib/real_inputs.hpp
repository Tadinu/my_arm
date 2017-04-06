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
// Filename      : real_inputs.hh
//
// Purpose       : definition of real inputs
//
// Special Notes :
//
//
// Creator       : Pier Luca Lanzi and Daniele Loiacono
//
// Creation Date :
//
// Modification  :
//
// Current Owner :
//
//-------------------------------------------------------------------------

/*!
 * \file real_inputs.hpp
 *
 * \brief implements real input values
 *
 */

/*!
 * \class real_inputs real_inputs.hpp
 *
 * \brief real_inputs implements the methods specific for real inputs
 *
 * \author
 *
 * \version
 *
 * \date
 *
 */


#ifndef __REAL_INPUTS__
#define __REAL_INPUTS__

#define __STATE_OPEN_CHAR__ '('
#define __STATE_CLOSE_CHAR__ ')'
#define __STATE_SEPARATOR_CHAR__ ','

#include <iostream>
#include <string>
#include <vector>

#include "inputs_base.hpp"
#include "xcs_utility.hpp"

using namespace std;

#define __REAL_INPUTS_VERSION__ "vector of reals (class long_sensors)"

class real_inputs : public inputs_base<real_inputs, double>
{

private:
	//! current sensor value
	vector<double> inputs;

	//! number of input values
	unsigned int no_inputs;

public:

	//! constructor
	real_inputs() {no_inputs = 1; inputs.reserve(1);};

	real_inputs(const real_inputs& ls);

	real_inputs(string value) { no_inputs = 0; inputs.clear(); set_string_value(value);};

	real_inputs(unsigned long dim) { no_inputs = dim; inputs.reserve(dim);};

	//! destructor
	virtual ~real_inputs() { inputs.clear(); }

	//! name of the class that implements the sensors
	string class_name() { return ("real_inputs"); };

	//! return the sensor size; for usual long sensors, returns the number of bits.
	unsigned long size() const {return no_inputs;};

	//! return the value of the specified input position
	double input(unsigned long) const;

	//! set the value of a specific input
	void set_input(unsigned long, double);

	//! set the value of the sensors from a string
	string string_value() const;

	//! set the value of the sensors from a string
	void set_string_value(const string &str);

	//! set no_inputs of input
	void set_dim(unsigned long dim)
	{
		no_inputs = dim;
                inputs.clear();
                inputs.reserve(dim);
	};

	//! equality operator
	bool operator==(const real_inputs& sens) const;

	//! not equal operator
	bool operator!=(const real_inputs& sens) const;

	//! assignment operators
	real_inputs& operator=(real_inputs& sens);
	real_inputs& operator=(const real_inputs& sens);

	double get_vect_length() const
	{
		double result=0;
		for (int i=0; i< no_inputs; i++)
			result+=inputs[i]*inputs[i];
		return result;
	}

	//! return true if the sensory inputs can be represented as a vector of long
	bool allow_numeric_representation() const { return true; };

	//! return a vector of long that represents the sensory inputs
	void numeric_representation(vector<double>&) const;

	//! set the sensory inputs as a vector of long
	void set_numeric_representation(const vector<double>& value);

};
#endif

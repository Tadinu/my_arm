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
// Filename      : binary_inputs.hh
//
// Purpose       : definition of state with binary representation
//                 
// Special Notes : 
//                 
//
// Creator       : Pier Luca Lanzi
//
// Creation Date : 2002/06/10
// 
// Modification  : 2002/08/25 all specialized methods are non virtual
//                 2004/09/27 changed from binary_sensors to binary_inputs
//
// Current Owner : Pier Luca Lanzi
//
//-------------------------------------------------------------------------

/*!
 * \file binary_inputs.h
 *
 * \brief implements binary state
 *
 */

/*!
 * \class binary_inputs binary_inputs.h
 *
 * \brief binary_inputs implements the methods specific for binary states
 *
 * \author Pier Luca Lanzi
 *
 * \version 0.01
 *
 * \date 2002/06/10
 *
 */

#include <iostream>
#include <string>
#include "inputs_base.hpp"
#include "xcs_utility.hpp"

#ifndef __BINARY_INPUTS__
#define __BINARY_INPUTS__

//! help string for the class
const string __INPUTS_VERSION__ = "binary {0,1} string (class binary_inputs)";

using namespace std;
//using namespace xcs;

class binary_inputs : public inputs_base<binary_inputs, char>
{
private:
	//! current state value
	string	value;

public:
	//! constructor
	binary_inputs() {};
	binary_inputs(string value) { set_string_value(value);};

	//! destructor
	virtual ~binary_inputs() {};

	//! name of the class that represents the state
	string class_name() { return ("binary_inputs"); };

	//! return the size of the state; for instance, with binary representation, returns the number of bits.
	unsigned long	size() const {return value.size();};

	//! set the value of the state from a string
	string string_value() const { return value; };

	//! set the value of the state from a string
	void set_string_value(const string &str);

	//! return the value of the specified input bit
	char input(unsigned long) const;

	//! set the value of a specific input
	void set_input(unsigned long, char);

	//! equality operator 
	bool operator==(const binary_inputs& st) const {
		return (string_value()==st.string_value());
	};

	//! not equal operator
	bool operator!=(const binary_inputs& st) const {
		return (string_value()!=st.string_value());
	};

	//! assignment operators
	binary_inputs& operator=(binary_inputs& st) { value = st.string_value(); return (*this); };
	binary_inputs& operator=(const binary_inputs& st) { value = st.string_value(); return (*this); };

	//! return true if the sensory inputs can be represented as a vector of long
	bool allow_numeric_representation() const { return true; };

	//! return a vector of long that represents the sensory inputs
	void numeric_representation(vector<long>&) const;
	void numeric_representation(vector<double>&) const;

	//! return a vector of long that represents the sensory inputs
	void set_numeric_representation(const vector<long>&);
	void set_numeric_representation(const vector<double>&);
};
#endif

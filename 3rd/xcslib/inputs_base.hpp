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
// Filename      : inputs_base.hpp
//
// Purpose       : base template class that specifies the basic methods 
//                 required to implement inputs
//                 
// Special Notes : 
//                 
//
// Creator       : Pier Luca Lanzi
//
// Creation Date : 2002/05/28
//
// Modifications : 
//          2002/12/02
// 		   added the methods allow_numeric_representation() and 
//		   numeric_representation(vector<double>&) to speed up the use of 
//		   symbolic and interval conditions
//          2004/09/25
//          changed to state from state
//
// Current Owner : Pier Luca Lanzi
//-------------------------------------------------------------------------


/*!
 * \file inputs_base.hpp
 *
 * \brief Base class to define sensory inputs, i.e., states.
 *
 */

/*!
 * \class inputs_base
 * \brief Base class to define sensory inputs, i.e., states.
 */

#ifndef __XCS_INPUTS_BASE__
#define __XCS_INPUTS_BASE__

#include <cassert>
#include <iostream>
#include <string>
#include <vector>

using namespace std;

template <class _inputs, class _input>
class inputs_base
{
public:
	/*!
	 * \fn string class_name() 
	 * \brief name of the class that implements the environment
	 * 
	 * This function returns the name of the class. 
	 * Every class must implement this method that is used to 
	 * trace errors.
	 */

	//! name of the class that implements the environment
	virtual string class_name() = 0;
	
	//! tag used to access the configuration file
	virtual string tag_name() { return ""; };

	//! return the size of the input state; for instance, for usual binary sensors, it returns the number of bits.
	virtual unsigned long	size() const = 0;

	//! return the value of a specific input
	virtual _input input(unsigned long) const = 0;

	//! set the value of a specific input
	virtual void set_input(unsigned long, _input) = 0;

	//! set the value of the sensors from a string
	virtual string string_value() const = 0;

	//! set the value of the sensors from a string
	virtual void set_string_value(const string &str) = 0;

	//! equality operator 
	virtual bool operator==(const _inputs& st) const = 0;

	//! not equal operator
	virtual bool operator!=(const _inputs& st) const = 0;
	
	//! assignment operators
	virtual _inputs& operator=(_inputs&) = 0;
	virtual _inputs& operator=(const _inputs&) = 0;

	//! write the sensors to an output stream 
	friend ostream& operator<<(ostream& output, const _inputs& st) {
		output << st.string_value();
		return (output);
	};

	//! read the sensors from an input stream 
	friend istream& operator>>(istream& input, _inputs& st) {
		string	str;
		input >> str;
		st.set_string_value(str);
		return (input);
	};

	//! return true if the sensory inputs can be represented as a vector of long
	virtual bool allow_numeric_representation() const { return false; };

	//! return a vector of values that represents the sensory inputs
	virtual void numeric_representation(vector<double>& vals) const {assert(false);};
	
	//! assign the values of the state values from a numerical vector
	virtual void set_numeric_representation(const vector<double>& vals) {assert(false);};
};
#endif

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
// Filename      : integer_action.hh
//
// Purpose       : definition of the integer actions class
//
// Special Notes :
//
//
// Creator       : Pier Luca Lanzi
//
// Creation Date : 2002/06/10
//
// Current Owner : Pier Luca Lanzi
//
//-------------------------------------------------------------------------

/*!
 * \file integer_action.hpp
 *
 * \brief implements integer actions used for instance in woods environments
 *
 */

/*!
 * \class integer_action integer_action.hh
 *
 * \brief implements integer actions used for instance in woods environments
 * \sa action_base
 *
 * \author 	Pier Luca Lanzi
 *
 * \version 	1.00
 *
 * \date 	2008/08/15
 *
 */

#define __ACTION_VERSION__ "integer (class integer_action)"

#ifndef __INTEGER_ACTION__
#define __INTEGER_ACTION__

#include "action_base.hpp"
#include "rl_definitions.hpp"
// #include "xcs_utility.h"
#include "xcs_config_mgr2.hpp"

class integer_action : public virtual action_base<integer_action>
{

private:
	static bool init;			//!< true if the class has been already inited
	static unsigned long no_actions;	//!< number of available actions

public:
	/*!
	 * \fn string class_name() const
	 * \brief name of the class that implements the environment
	 *
	 * This function returns the name of the class.
	 * Every class must implement this method that is used to
	 * trace errors.
	 */
	//! name of the class that implements the environment
	string class_name() const { return string("integer_action"); };

	//! tag used to access the configuration file
	string tag_name() const { return string("action::integer"); };

	//! constructor that reads the class parameters through the configuration manager
	/*!
	 *  This is the first constructor that must be used. Otherwise an error is returned.
	 */
	integer_action(xcs_config_mgr2&);

	//! constructor that sets the class parameters inline
	integer_action(int act);

	//! default constructor that can be used only after the class has been already initialized through the configuration manager
	integer_action();

	//! return the number of available actions
	unsigned long actions() const {return no_actions;};

	//! mutate the action according to the mutation rate \emph mu
	void mutate(const double&);

	//! return the action value as a string
	string string_value() const;

	//! set the action value from a string
	void set_string_value(string);

	//! assignment operator
	virtual integer_action& operator=(integer_action& action) {set_string_value(action.string_value()); return *this;};

	//! assignment operator for a constant value
	virtual integer_action& operator=(const integer_action& action) {set_string_value(action.string_value()); return *this;};

};
#endif

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
// Filename      : boolean_action.hh
//
// Purpose       : class that implements Boolean actions
//                 
// Special Notes : 
//
// Creator       : Pier Luca Lanzi
//
// Creation Date : 2002/05/27
//
// Last Change   : 
//
// Current Owner : Pier Luca Lanzi
//-------------------------------------------------------------------------

/*! 
 *  \file boolean_action.hh
 *  \brief class for boolean (i.e., 0/1) actions
 *
 * \author Pier Luca Lanzi
 *
 * \version 0.01
 *
 * \date 2002/05/27
 * 
 */


#ifndef __BOOLEAN_ACTION__
#define __BOOLEAN_ACTION__

#define __ACTION_VERSION__ "Boolean value 0/1 (class boolean_action)"

#include "action_base.hpp"
#include "xcs_config_mgr2.hpp"

/*!
 * \class boolean_action
 *
 * \brief definition of Boolean actions
 * \sa action_base
 *
 */

class boolean_action : public virtual action_base<boolean_action>
{
public:
	/*!
	 * \fn string class_name() const
	 * \brief name of the class that implements the action set
	 * 
	 * This function returns the name of the class. 
	 * Every class must implement this method that is used to 
	 * trace errors.
	 */
	string class_name() const { return string("boolean_action"); };

	//! tag used to access the configuration file
	string tag_name() const { return string("boolean_action"); };

	//! default constructor sets the action to 0 (i.e., false)
	boolean_action();

	//! creates an action with the specified value
	boolean_action(int);

	//! init the action parameters from file. In this case, nothing is done.
	boolean_action(xcs_config_mgr2&);

	//! mutate the action according to the mutation rate
	void mutate(const double&);

	//! specify that Boolean actions can have two values only (0 and 1).
	unsigned long actions() const {return 2;};

	//! return the action value as a string
	string string_value() const;

	//! set the action value from a string
	void set_string_value(string str);
};
#endif

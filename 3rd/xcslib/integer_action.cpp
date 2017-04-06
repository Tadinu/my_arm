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
// Filename      : integer_action.cpp
//
// Purpose       : implementation of the integer actions class
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
 * \file integer_action.cpp
 *
 * \brief implements integer actions used for instance in woods environments
 *
 */

#include <sstream>
#include <string>
#include "xcs_random.hpp"
#include "integer_action.hpp"

unsigned long	integer_action::no_actions;

bool integer_action::init = false;

integer_action::integer_action()
{
	if (!integer_action::init)
	{
		xcs_utility::error(class_name(),"integer_action()", "not inited", 1);
	} else {

	}
}

integer_action::integer_action(int act)
{
	if (!integer_action::init)
	{
		xcs_utility::error(class_name(),"integer_action(int)", "not inited", 1);
	} else {
		action = act;
		integer_action::init=true;
	}
}

integer_action::integer_action(xcs_config_mgr2& xcs_config)
{
	string		input_configuration;
	
	if (!integer_action::init)
	{

		//! look for the init section in the configuration file
		if (!xcs_config.exist(tag_name()))
		{
			xcs_utility::error(class_name(), "constructor", "section <" + tag_name() + "> not found", 1);	
		}
	
		try {
			no_actions = xcs_config.Value(tag_name(), "number of actions");
			integer_action::init=true;
		} catch (const char *attribute) {
			string msg = "attribute \'" + string(attribute) + "\' not found in <" + tag_name() + ">";
			xcs_utility::error(class_name(), "constructor", msg, 1);
		}
	} else {
		xcs_utility::error(class_name(),"integer_action(xcs_config_mgr)", "already inited", 1);
	}
}

/*
integer_action::integer_action(xcs_config_mgr2& xcs_config)
{
	string		input_configuration;
	
	if (!integer_action::init)
	{
		init = true;

		if (!xcs_config.exist(tag_name()))
		{
			xcs_utility::error(class_name(), "constructor", "section <" + tag_name() + "> not found", 1);	
		}

		try {
			
			no_actions = xcs_config.Value(tag_name(), "number of actions");

		} catch (const char *attribute) {
			string msg = "attribute \'" + string(attribute) + "\' not found in <" + tag_name() + ">";
			xcs_utility::error(class_name(), "constructor", msg, 1);
		}

		integer_action::init=true;

	} else {
		xcs_utility::error(class_name(),"integer_action(xcs_config_mgr)", "already inited", 1);
	}
}*/

void
integer_action::mutate(const double& mu)
{		
	if (xcs_random::random()<mu)
	{
		action=xcs_random::dice(integer_action::no_actions);
	}
}

string 
integer_action::string_value() const
{	
	ostringstream sstr;
	string	  str;
	sstr << action;

	str = sstr.str();
	
	return str;
	//! in C it would be implemented as:
	//  char str[10];
	//  sprintf(str, "%d", action);
	//  return string(str);
}

void 
integer_action::set_string_value(string str)
{
	action = atoi(str.c_str()) % actions();
}

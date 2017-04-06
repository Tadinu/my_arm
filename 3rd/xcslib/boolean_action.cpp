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
// Filename      : boolean_action.cc
//
// Purpose       : implement Boolean actions
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
 *  \file boolean_action.cc
 *  \brief class implementation for Boolean (i.e., 0/1) actions
 *
 * \author Pier Luca Lanzi
 *
 * \version 0.01
 *
 * \date 2002/05/27
 * 
 */

#include "xcs_random.hpp"
#include "boolean_action.hpp"

boolean_action::boolean_action()
{
	action = 0;
};

boolean_action::boolean_action(int act)
{
	if ((act!=1)&&(act!=0))
	{
		xcs_utility::error(class_name(),"class constructor", "value not allowed", 1);
	}
	action = act;
};

boolean_action::boolean_action(xcs_config_mgr2& xcs_config)
{
	action = 0;
};

void
boolean_action::mutate(const double& mu)
{		
	if (xcs_random::random()<mu)
	{
		action=1-action;
	}
}

string 
boolean_action::string_value() const
{
	if (action) 
		return string("1"); 
	else 
		return string("0");
}

void 
boolean_action::set_string_value(string str)
{
	if (str=="0")
		action = 0;
	else if (str=="1")
		action = 1;
	else {
		xcs_utility::error(class_name(),"set_string_value", "value '"+str+"' not allowed", 1);
	}
}

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
// Filename      : binary_action.hh
//
// Purpose       : class that implements binary actions
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
 * \file 	binary_action.hpp
 * \brief 	class for binary actions
 * \author 	Pier Luca Lanzi
 * \version 	1.0
 * \date 	2008/08/06
 */
 
#include "action_base.hpp"
#include "xcs_utility.hpp"
#include "xcs_config_mgr2.hpp"

#ifndef __BINARY_ACTION__
#define __BINARY_ACTION__

#define __ACTION_VERSION__ "binary string (class binary_action)"

using namespace std;

class binary_action : public virtual action_base<binary_action>
{
		
private:
	static bool				init;
	static unsigned long	no_actions;
	static unsigned long	no_bits;
	string					bitstring;

public:
	string class_name() const { return string("binary_action"); };
	string tag_name() const { return string("action::binary"); };

	binary_action();
	binary_action(int);
	binary_action(xcs_config_mgr2&);

	~binary_action();
	

	unsigned long actions() const;

	void set_value(unsigned long);
	string string_value() const;
	void set_string_value(string);

	void random();
	void mutate(const double&); 
};

#endif

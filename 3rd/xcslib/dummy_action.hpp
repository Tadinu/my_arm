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




#include "action_base.hpp"
#include "xcs_utility.hpp"
#include "xcs_config_mgr2.hpp"

#ifndef __DUMMY_ACTION__
#define __DUMMY_ACTION__

#define __DUMMY_ACTION_PRINT__ "#";
// #define __ACTION_VERSION__ "dummy (class dummy_action)"


const string __ACTION_VERSION__ = "dummy action (class dummy action)";

class dummy_action : public virtual action_base<dummy_action>
{

public:
	string class_name() const { return string("dummy_action"); };
	string tag_name() const { return string("dummy_action"); };

	dummy_action();

	dummy_action(xcs_config_mgr2&){};

	dummy_action(int){};

	~dummy_action(){};

	unsigned long actions() const {return 1;};

	//! return the integer action value
	unsigned long value() const { return 0; };

	void set_value(unsigned long){};

	string string_value() const {return __DUMMY_ACTION_PRINT__;};

	void set_string_value(string){};

        //! equal operator
        bool operator==(const dummy_action& act) const { return true; };

        //! less than operator
        virtual bool operator< (const dummy_action& act) const { return false; };

        //! not equal operator
        virtual bool operator!=(const dummy_action& act) const { return false; };

	void random(){};

	void mutate(const double& mu){};

};

#endif

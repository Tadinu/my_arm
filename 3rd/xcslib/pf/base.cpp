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




/*
 * The XCS Library 
 * A C++ framework to apply and develop learning classifier systems
 * Copyright (C) 2002-2008 Pier Luca Lanzi and Daniele Loiacono
 * 
 * Pier Luca Lanzi and Daniele Loiacono
 * Dipartimento di Elettronica e Informazione
 * Politecnico di Milano
 * Piazza Leonardo da Vinci 32
 * I-20133 MILANO - ITALY
 * pierluca.lanzi@polimi.it
 * loiacono@elet.polimi.it
 *
 *  This file is part of the XCSLIB library.
 *
 *  xcslib is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  xcslib is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 * 
 * A copy of the license is available at http://www.gnu.org/licenses/lgpl.html
 */




#include <sstream>
#include "pf/base.hpp"
#include "xcs_utility.hpp"
#include "xcs_random.hpp"
//#include "xcs_classifier.hpp"

using namespace std;

namespace xcsflib
{

//! init static variables
bool		base_pf::init = false;		//! true when base function has been inited
unsigned long	base_pf::dimension;		//! number of inputs
t_mutation_type base_pf::mutation_type;		//! mutation type (not used)
t_init_mode	base_pf::init_mode;		//! init procedure for parameter vector
t_function_type base_pf::default_function;	//! default type of prediction function

base_pf::base_pf( xcs_config_mgr2& xcs_config)
{
	string	str_mutation;			//! mutation type
	string	str_function;			//! default function
	string	str_init_mode;			//! string to set init mode

	if (!xcs_config.exist(tag_name()))
	{
		xcs_utility::error(class_name(), "constructor", "section <" + tag_name() + "> not found", 1);	
	}

	try {
		base_pf::dimension = xcs_config.Value(tag_name(), "input size");
		str_function = (string) xcs_config.Value(tag_name(), "prediction function");
		str_init_mode = (string) xcs_config.Value(tag_name(), "init mode", "default");
	} catch (const char *attribute) {
		string msg = "attribute \'" + string(attribute) + "\' not found in <" + tag_name() + ">";
		xcs_utility::error(class_name(), "constructor", msg, 1);
	}

	//! get the default prediction function type
	default_function = get_function_type(str_function);

	//! how parameter vectors are initialized
	init_mode = get_init_mode(str_init_mode);

	init = true;

}

} //! end of xcsflib namespace

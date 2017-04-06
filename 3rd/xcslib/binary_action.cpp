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




#include <string>
#include <iostream>
#include <cmath>
#include "binary_action.hpp"
#include "xcs_random.hpp"
#include "xcs_utility.hpp"

using namespace std;

const char* __XCS_BITSTRING_ACTION_CFG_IN__ =	"\n number of bits = %u ";
const char* __XCS_BITSTRING_ACTION_CFG_OUT__ =	"\t\tnumber of bits = %u\n";
#define     __XCS_BITSTRING_VARS_IN__ binary_action::no_bits

bool binary_action::init = false;
unsigned long binary_action::no_actions;
unsigned long binary_action::no_bits;

binary_action::binary_action()
{
	if (!init)
	{
		xcs_utility::error(class_name(),"binary_action()", "not inited", 1);
	} else {

	}
}

binary_action::binary_action(int act)
{
	if (!init)
	{
		xcs_utility::error(class_name(),"binary_action()", "not inited", 1);
	} else {
		action = act;
		bitstring = xcs_utility::long2binary(act, binary_action::no_bits);
	}
}


unsigned long
binary_action::actions() const
{
	return binary_action::no_actions;
};


binary_action::binary_action(xcs_config_mgr2& xcs_config)
{
	if (!init)
	{
		if (!xcs_config.exist(tag_name()))
		{
			xcs_utility::error(class_name(), "constructor", "section <" + tag_name() + "> not found", 1);	
		}
	
		try {
			
			no_bits = xcs_config.Value(tag_name(), "number of bits");

		} catch (const char *attribute) {
			string msg = "attribute \'" + string(attribute) + "\' not found in <" + tag_name() + ">";
			xcs_utility::error(class_name(), "constructor", msg, 1);
		}

		init = true;
		
		no_actions = (unsigned long) pow(double(2),int(binary_action::no_bits));

#ifdef __DEBUG__
		cout << "BITS " << binary_action::no_bits << endl;
		cout << "ACTIONS " << actions() << endl;
#endif
	}
};

binary_action::~binary_action()
{

}

void
binary_action::random()
{		
	string::size_type	bit;

	bitstring = "";

	for(bit=0; bit<binary_action::no_bits; bit++)
	{
		if (xcs_random::random()<.5)
			bitstring += "1";
		else
			bitstring += "0";
	}
	
	action = xcs_utility::binary2long(bitstring);
}

void
binary_action::mutate(const double& mutationRate)
{		
	string::size_type	bit;

	for(bit=0; bit<bitstring.size(); bit++)
	{
		if (xcs_random::random()<mutationRate)
		{
			if (bitstring[bit]=='0')
				bitstring[bit] = '1';
			else
				bitstring[bit] = '0';
		}
	}
	action = xcs_utility::binary2long(bitstring);
}

string 
binary_action::string_value() const
{
	return bitstring;
}

void 
binary_action::set_string_value(string str)
{
	bitstring = str;
	action = xcs_utility::binary2long(str);
}
/*
binary_action&
binary_action::operator=(binary_action& act)
{
	action = act.action; 
	xcs_utility::long2binary(act,no_bits);
	return *this;
}; 

binary_action&
binary_actiono::operator=(const binary_action& act)
{
	action = act.action; 
	xcs_utility::long2binary(act,no_bits);
	return *this;
}*/
void 
binary_action::set_value(unsigned long act) { 
	action = act;
	bitstring = xcs_utility::long2binary(act, binary_action::no_bits);
};

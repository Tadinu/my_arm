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
// Filename      : parity_env.cc
//
// Purpose       : implementation of the parity class 
//                 
// Special Notes : 
//                 
// Creator       : Pier Luca Lanzi
//
// Creation Date : 2002/05/31
//
// Current Owner : Pier Luca Lanzi
//
//-------------------------------------------------------------------------

/*!
 * \file parity_env.cc
 *
 * \brief implements the Boolean hidden parity
 *
 * \author Pier Luca Lanzi
 *
 * \version 0.01
 *
 * \date 2005/08/01
 *
 */

#include <sstream>
#include <cmath>
#include "parity_env.hpp"


using namespace std;

//!< set the init flag to false so that the use of the config manager becomes mandatory
bool	parity_env::init=false;	

parity_env::parity_env(xcs_config_mgr2& xcs_config)
{
	ifstream 	config;
	string 		str_input;
	
	if (!parity_env::init)
	{
		parity_env::init = true;

		if (!xcs_config.exist(tag_name()))
		{
			xcs_utility::error(class_name(), "constructor", "section <" + tag_name() + "> not found", 1);	
		}
		
		xcs_config.save(cerr);
		
		try {
			string_size = xcs_config.Value(tag_name(), "size");
			str_input = (string) xcs_config.Value(tag_name(), "input selection");
		} catch (const char *attribute) {
			string msg = "attribute \'" + string(attribute) + "\' not found in <" + tag_name() + ">";
			xcs_utility::error(class_name(), "constructor", msg, 1);
		}

		state_size = string_size;
		read_selected_variables((char*) str_input.c_str(), selected_inputs);

		no_configurations = 1;
		no_configurations <<= state_size;

#ifdef __DEBUG__
		cout << "STATE SIZE" << state_size << endl;
		cout << "STR " << str_input << endl;
		cout << "SELECTED INPUTS" << endl;
		for(unsigned long var=0; var<selected_inputs.size(); var++)
		{
			cout << "\t" << var << ".\t" << selected_inputs[var] << endl;
		}
		cout << "INPUT SIZE    " << string_size << endl;
#endif
	}

	parity_env::init = true;
}

/*!
 * \fn void parity_env::begin_problem(const bool explore)
 * \param explore true if the problem is solved in exploration
 *
 * \brief generates a new input configuration for the Boolean parity
 *
 * If the inputs must be visited uniformly, indicated by uniform_start set to true,
 * the variable current_configuration is used to generate the next available input;
 * otherwise, a random input configuration is generated
 */
void	
parity_env::begin_problem(const bool explore)
{
	current_reward = 0;

	string	str;

	string::size_type	bit;

	str = "";

	for(bit = 0; bit<state_size; bit++)
	{
		str += '0' + xcs_random::dice(2);
	}

	inputs.set_string_value(str);
	current_reward = 0;
	first_problem = false;
}

bool	
parity_env::stop()
const
{
	return(true); 
}


void	
parity_env::perform(const t_action& action)
{
	string			str_inputs;
	unsigned long		in;
	unsigned long		sum;
	unsigned long		result;
	
	str_inputs = inputs.string_value();

	sum = 0;

	for(in=0; in<selected_inputs.size(); in++)
	{
		sum += str_inputs[selected_inputs[in]]-'0';
	}

	if (sum%2)
		result = 0;
	else 
		result = 1;

	if (result==action.value())
	{
		current_reward = 1000;
	} else {
		current_reward = 0;
	}
#ifdef __DEBUG__
	cout << "INPUT " << inputs << " BIT " << sum%2 << " ACTION " << action.value() << " REWARD " << current_reward << endl;
#endif
}

//! only the current reward is traced
void
parity_env::trace(ostream& output) const
{
	output << current_reward;
}

void 
parity_env::reset_input()
{
	current_state = 0;
	inputs.set_string_value(xcs_utility::long2binary(current_state,state_size));
}

bool 
parity_env::next_input()
{
	string	binary;
	bool	valid = false;

	current_state++; 
	if (current_state<no_configurations)
	{
		inputs.set_string_value(xcs_utility::long2binary(current_state,state_size));
		valid = true;
	} else {
		current_state = 0;
		inputs.set_string_value(xcs_utility::long2binary(current_state,state_size));
		valid = false;
	}
	return valid;
}

void
parity_env::save_state(ostream& output) const
{
	output << endl;
	output << current_state << endl;
}

void
parity_env::restore_state(istream& input)
{
	input >> current_state; 
	begin_problem(true);
}

parity_env::parity_env()
{
	if (!parity_env::init)
	{
		xcs_utility::error(class_name(),"class constructor", "not inited", 1);
	} else {
		// nothing to init
	}
}

void
parity_env::read_selected_variables(char *str, vector<unsigned long> &vars)
const
{
	long		var;
	long		n;
	char		comma;
	istringstream	intervals(str);

	//cout << "SUBSTR <" << (string(str).substr(0,7)) << ">" << endl;

	if (string(str)=="all")
	{
		for(var=0; var<state_size; var++)
			vars.push_back(var);
	} else if (string(str)=="lograndom") {
		n = long(ceil(log(double(state_size))/log(2.0)));
		for(var=0; var<n; var++)
			vars.push_back(xcs_random::dice(state_size));
	} else if (string(str).substr(0,7)=="random:") {
		sscanf(str,"random:%ld",&n);
		cout << "\t" << n << " RANDOM VARIABLES" << endl;
		cout << "\t" << state_size << " RANDOM VARIABLES" << endl;
		if (n>state_size)
		{
			xcs_utility::error(class_name(),"class constructor", "too many selected variables", 1);
		}
		for(var=0; var<n; var++)
			vars.push_back(xcs_random::dice(state_size));
	} else {
		vars.clear();
		if(intervals>>var)
		{
			if (var>=state_size)
			{
				xcs_utility::error(class_name(),"class constructor", "selected variable out of bound", 1);
			}
			vars.push_back(var);
			while (intervals>>comma>>var)
			{
				if (var>=state_size)
				{
					xcs_utility::error(class_name(),"class constructor", "selected variable out of bound", 1);
				}
				vars.push_back(var);
			}
		}
	}
	if (vars.size()==0)
	{
		xcs_utility::error(class_name(),"class constructor", "no variables have been selected", 1);
	}
}

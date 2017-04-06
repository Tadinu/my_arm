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
// Filename      : fsm_env.cpp
//
// Purpose       : implements a finite state world
//                 
// Special Notes : 
//                 
//
// Creator       : Pier Luca Lanzi
//
// Creation Date : 2005/02/09
//
// Current Owner : Pier Luca Lanzi
//-------------------------------------------------------------------------

/*!
 * \file fsm_env.cpp
 *
 * \brief implementation of a 7 Step Finite State World
 *
 */
#include "fsm_env.hpp"

bool	fsm_env::init = false;

fsm_env::fsm_env(xcs_config_mgr2& xcs_config)
{
	if (!fsm_env::init)
	{
		if (!xcs_config.exist(tag_name()))
		{
			xcs_utility::error(class_name(), "constructor", "section <" + tag_name() + "> not found", 1);	
		}
		
		try {
			state_bits    = (long) xcs_config.Value(tag_name(), "number of bits");
			states_number = (long) xcs_config.Value(tag_name(), "number of states");
		} catch (const char *attribute) {
			string msg = "attribute \'" + string(attribute) + "\' not found in <" + tag_name() + ">";
			xcs_utility::error(class_name(), "constructor", msg, 1);
		}

		//! look for the init section in the configuration file
		string input_configuration;
		
		if ((states_number%2) == 0)			
			xcs_utility::error(class_name(),"class constructor", "Number of states must be even", 1);
		
		current_state = 0;
		final_state = (states_number-1)/2;
	
		set_state();
	}
	fsm_env::init = true;
}

void	
fsm_env::begin_problem(const bool explore)
{
	ostringstream	PATH;

	//! at the beginning of the problem the previous information about the agent's path is cleared
	path = "";
	
	//! at the beginning 0 is the start position
	current_state=0;		
	set_state();

	PATH << current_state << "\t";
	path += PATH.str();
}

bool	
fsm_env::stop()
const
{
	return(current_state==final_state); 
}


void	
fsm_env::perform(const t_action& action)
{	
	//! in the lower part of chain all the actions leads to an upper state
	if (current_state > final_state)
	{
		current_state -= final_state;
	}
	else //! in the upper state, destination depend on the action and on the current state
	{
		if (current_state % 2)
		{
			int	inc_state[] = { 1 , final_state+1 };
			current_state += inc_state[action.value()];
		}
		else
		{
			int	inc_state[] = { final_state+1 , 1 };
			current_state += inc_state[action.value()];
		}		
	}
	
	set_state();
	
	ostringstream PATH;
	PATH << current_state << "\t";
	path += PATH.str();
}

void
fsm_env::trace(ostream& output) const
{
	output << path;
}

void 
fsm_env::reset_input()
{
	reset_problem();
}

bool 
fsm_env::next_input()
{
	return next_problem();
}

void
fsm_env::save_state(ostream& output) const
{
	output << endl;
	output << current_state << endl;	
}

void
fsm_env::restore_state(istream& input)
{
	input >> current_state;	
	set_state();
}


inline
void	
fsm_env::set_state()
{	
	if ( current_state == final_state)
	{
		current_reward = 1;
	} else {
		current_reward = 0;
	}
	
	long decimal = current_state;
	string binary;
	for (int i=0; i<state_bits; i++)
	{
		unsigned int bit;
		bit = decimal % 2;
		decimal = floor((double)decimal/2);
		if (bit==0)
			binary = "0" + binary;
		else
			binary = "1" + binary;
	}	
	
	inputs.set_string_value(binary);	
}

void
fsm_env::reset_problem()
{
	path = "";

	current_state = 0;
	set_state();
	
	ostringstream PATH;
	PATH << current_state << "\t";
	path = PATH.str();
}


bool
fsm_env::next_problem()
{
	reset_problem();
	return false;
}

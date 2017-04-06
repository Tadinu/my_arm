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
// Filename      : parity_env.hh
//
// Purpose       : definition of the class for the hidden parity problem 
//                 
// Special Notes : 
//                 
//
// Creator       : Pier Luca Lanzi
//
// Creation Date : 2002/05/14
//
// Current Owner : Pier Luca Lanzi
//
//-------------------------------------------------------------------------

/*!
 * \file parity_env.hh
 *
 * \brief implements the Boolean parity
 *
 */

/*!
 * \class parity_env parity_env.hh
 *
 * \brief parity class that implements the methods specific for the Boolean parity 
 * \sa environment_base
 *
 * \author Pier Luca Lanzi
 *
 * \version 0.01
 *
 * \date 2002/10/30
 *
 */

#ifndef __PARITYENV__
#define __PARITYENV__

#include <cassert>
#include "rl_definitions.hpp"
#include "environment_base.hpp"
#include "xcs_config_mgr2.hpp"

using namespace std;

using namespace std;

class parity_env : public virtual environment_base
{
 public:
	string class_name() const { return string("parity_env"); };
	string tag_name() const { return string("environment::parity"); };
		
	//! Constructor for the parity class that read the class parameters through the configuration manager
	/*!
	 *  This is the first constructor that must be used. Otherwise an error is returned.
	 */
	parity_env(xcs_config_mgr2&);

	//! Default constructor for the Boolean parity class
	/*!
	 *  If the class parameters have not yet initialized through the configuration manager, 
	 *  an error is returned and the method exists to shell. 
	 *  \sa parity_env(xcs_config_mgr&)
	 *  \sa xcs_config_mgr
	 */
	parity_env();
	
	void begin_experiment() {};
	void end_experiment() {};

	void begin_problem(const bool explore);
	void end_problem() {};

	bool stop() const;
	
	void perform(const t_action& action);

	void trace(ostream& output) const;

	void reset_input();
	bool next_input();

	void save_state(ostream& output) const;
	void restore_state(istream& input);
	
	virtual double reward() const {assert(current_reward==parity_env::current_reward); return current_reward;};

	virtual t_state state() const { return inputs; };

 private:
	/*! \var bool init 
	 *  \brief true if the class parameters have been already initialized
	 */
	static bool			init;

	/*! 
	 * \var t_state inputs
	 * \brief inputs current input configuration
	 */
	t_state			inputs;			// input configuration

	/*! 
	 * \var bool first_problem 
	 * \brief true if the first problem is running
	 */
	bool				first_problem;

	/*! \var unsigned long string_size
	 * \brief number of string bits for the Boolean parity
	 */
	unsigned long			string_size;

	//! selected variables
	vector<unsigned long>		selected_inputs;

	//! true if it the system must visit all the available configuration as a sequence
	bool				uniform_start;		

	//! size of the whole parity string, e.g., 6 for the 6-way parity
	unsigned long			state_size;

	//! the reward returned as a consequence of the last performed action
	double				current_reward;

 private:
	//! read selected variables
	void read_selected_variables(char*, vector<unsigned long>&) const;
};
#endif

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
// Filename      : multiplexer_env.hpp
//
// Purpose       : definition of the multiplexer class 
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
// Modification  : 
//                 2005/07/25 modified the interface to configuration
//                 2003/02/22
//                 added the options for layered reward, deleted the flag for uniform start
//		   2003/08/06
//		   general.hpppp becomes xcs_definitions.hpppp
//
//-------------------------------------------------------------------------

/*!
 * \file multiplexer_env.hpp
 *
 * \brief implements the Boolean multiplexer
 *
 */

/*!
 * \class multiplexer_env 
 *
 * \brief multiplexer class that implements the methods specific for the Boolean multiplexer 
 * \sa environment_base
 *
 * \author Pier Luca Lanzi
 *
 * \version 0.01
 *
 * \date 2002/05/14
 *
 */


#ifndef __MULTIPLEXERENV__
#define __MULTIPLEXERENV__

#include <cassert>
#include "rl_definitions.hpp"
#include "environment_base.hpp"
#include "xcs_config_mgr2.hpp"

using namespace std;

class multiplexer_env : public virtual environment_base
{
 public:
	string class_name() const { return string("multiplexer_env"); };
	string tag_name() const { return string("environment::multiplexer"); };
		
	//! Constructor for the multiplexer class that read the class parameters through the configuration manager
	/*!
	 *  This is the first constructor that must be used. Otherwise an error is returned.
	 */
	multiplexer_env(xcs_config_mgr2&);

	//! Default constructor for the Boolean multiplexer class
	/*!
	 *  If the class parameters have not yet initialized through the configuration manager, 
	 *  an error is returned and the method exists to shell. 
	 *  \sa multiplexer_env(xcs_config_mgr&)
	 *  \sa xcs_config_mgr
	 */
	multiplexer_env();
	
	void begin_experiment() {};
	void end_experiment() {};

	void begin_problem(const bool explore);
	void end_problem() {};

	bool stop() const;
	
	void perform(const t_action& action);

	void trace(ostream& output) const;

	void reset_input();
	bool next_input();

	void reset_problem() {reset_input();};
	bool next_problem() {return next_input();};

	void save_state(ostream& output) const;
	void restore_state(istream& input);
	
	virtual double reward() const {assert(current_reward==multiplexer_env::current_reward); return current_reward;};

	virtual t_state state() const { return inputs; };

	bool allow_test() const {if (address_size<=3) return true; else return false;};
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

	/*! \var unsigned long address_size
	 * \brief number of address bits for the Boolean multiplexer
	 */
	unsigned long			address_size;

	//! true if it the reward is layered (Butz et al. GECCO 2001)
	bool				flag_layered_reward;		

	//! size of the whole multiplexer string, e.g., 6 for the 6-way multiplexer
	unsigned long			state_size;

	//! the reward returned as a consequence of the last performed action
	double				current_reward;

	//! true if the problem was solved correctly
	bool				solved;

};
#endif

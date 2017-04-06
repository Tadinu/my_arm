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
// Filename      : fsm_env.hh
//
// Purpose       : implements a 7 Step Finite State World
//                 
// Special Notes : 
//                 
//
// Creator       : Pier Luca Lanzi and Daniele Loiacono
//
// Creation Date : 2005/02/09
//
//
// Current Owner : Pier Luca Lanzi and Daniele Loiacono
//-------------------------------------------------------------------------

/*!
 * \file fsm_env.hh
 *
 * \brief implements a 7 Step Finite State World
 *
 */


#ifndef __FSM_ENV__
#define __FSM_ENV__

#include <sstream>

#include "rl_definitions.hpp"
#include "environment_base.hpp"
#include "xcs_config_mgr2.hpp"

/*!
 * \class fsm_env fsm_env.hpp
 * \brief implements the methods specific for chain environments
 * \sa environment_base
 */

class fsm_env : public virtual environment_base
{
public:
	string class_name() const { return string("fsm_env"); };
	string tag_name() const { return string("environment::fsm"); };
	
	//! Constructor for the chain environment class. It reads the class parameters through the configuration manager.
	/*!
	 *  This is the only constructor that can be used. 
	 */
	fsm_env(xcs_config_mgr2&);

	//! Destructor for the chain environment class.
	~fsm_env(){};
	
	void begin_problem(const bool explore);
	void end_problem() {};

	void begin_experiment() {};
	void end_experiment() {};

	bool stop() const;
	
	void perform(const t_action& action);

	void trace(ostream& output) const;

	bool allow_test() const {return true;};
	void reset_problem();
	bool next_problem();

	void reset_input();
	bool next_input();

	void save_state(ostream& output) const;
	void restore_state(istream& input);

	//! indicates that chain environments are multiple step problems
	virtual bool single_step() const {return false;};

 public:
	virtual double reward() const {assert(current_reward==fsm_env::current_reward); return current_reward;};
	virtual t_state state() const { return inputs; };
	virtual void print(ostream& output) const { output << current_state << "\t";};

 private:
 	
	//! given the current <x,y> position sets the current input and the current reward \sa current_position_x \sa current_position_y
	inline void	set_state();
	
	static bool			init;			//!< true if the class has been inited through the configuration manager
	t_state				inputs;			//!< current input configuration
	
	long 				states_number;		//! number of states in the environment
	long				state_bits;		//! number of bits to encode the states
	long				final_state;		//! final state in the environment

        //! current reward returned for the last action performed
	double		current_reward;
	//unsigned long	no_configurations;			// #configurazioni possibili

	//! \var current_configuration index of the current agent's input
	/*!
	 * it is used when scanning all the possible environment configurations with 
	 * the functions \fn reset_input and \fn next_input
	 * \sa reset_input
	 * \sa next_input
	 */
	
        //! current x position in the environment
	unsigned long 	current_state;
         	
        //! \var prob_slide specifies the probability that the agent can slip while it moves (Colombetti and Lanzi 1999)
	double		prob_slide; 
	
	//! path traces the path the agent followed during the problem
	string			path;
};
#endif

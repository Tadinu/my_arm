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
// Filename      : environment_base.hpp
//
// Purpose       : base template class that specifies the basic methods 
//                 required to implement an environment
//                 
// Special Notes : 
//                 
//
// Creator       : Pier Luca Lanzi
//
// Creation Date : 2002/05/13
//
// Current Owner : Pier Luca Lanzi
//-------------------------------------------------------------------------



/*!
 * \file environment_base.hpp
 *
 * \brief implements the base class to define environments
 *
 */

/*!
 * \class environment_base
 * \brief base class that specifies the basic methods required to implement an environment
 */

#ifndef __ENV_BASE_HH__
#define __ENV_BASE_HH__

#include "rl_definitions.hpp"

class environment_base
{
public:
	/*!
	 * \fn string class_name() const
	 * \brief name of the class that implements the environment
	 * 
	 * This function returns the name of the class. 
	 * Every class must implement this method that is used to 
	 * trace errors.
	 */
	//! name of the class that implements the environment
	virtual string class_name() const = 0;
	
	//! tag used to access the configuration file
	virtual string tag_name() const = 0;

	//! defines what has to be done when a new problem begins. 
	/*! 
	 * \param explore is true if the problem is done in exploration;
	 * it is false if the problem is solved in exploitation.
	 */
	virtual void begin_problem(const bool explore) = 0;

	//! defines what must be done when the current problem ends
	virtual void end_problem() = 0;

	//! performs action act in the environment 
	virtual void perform(const t_action& act) = 0;

	//! returns the current reward 
	virtual double reward() const = 0;

	//! returns the current state of the environment 
	virtual t_state state() const = 0;

	//! print the current state of the environment to an output stream. 
	/*!
	 * It is used as a way to pretty print the information about the environmental state;
	 * by default it output the result of the state function, but it can be overriden to 
	 * have richer output.
	 */
	virtual void print(ostream& output) const { output << state();};

	//! returns true if the problem has ended; it is used in multistep problems while it is always true in single step problems
	virtual bool stop() const = 0;
	
	//! writes trace information on an output stream; it is called just before the end_problem method \sa end_problem
	virtual void trace(ostream& output) const = 0;

	//! reset the environment on the first possible state
	virtual void reset_input() = 0;

	//! returns the next available environmental state; 
	virtual bool next_input() = 0;

	//! restore the state of the environment from an input stream
	virtual void restore_state(istream& input) = 0;

	//! save the state of the environment to an output stream
	virtual void save_state(ostream& output) const = 0;

	//! defines what has to be done when a new experiment begins
	virtual void begin_experiment() = 0;

	//! defines what has to be done when the current experiment begins
	virtual void end_experiment() = 0;

	//! returns true if the environment implements a single-step problem; false otherwise.
	/*!
	 *  the method can used by the experiment manager.
	 */
	virtual bool single_step() const {return true;};

	/*! \var unsigned long no_configurations
	 * \brief number of possible input configurations
	 */
	unsigned long			no_configurations;
	unsigned long			current_configuration;	//!<	current visited configuration 
	unsigned long			current_state;		//!<	current visited state
	virtual ~environment_base() {};

	//! methods to test environment
	//! true if the environment allows the test of all the possible configurations
	virtual bool allow_test() const {return false;};

	//! reset the environment on the first possible problem configuration
	/*!
	 * since by default the environment does not allow the test of all the problems, this methods does nothing
	 */
	virtual void reset_problem() {};

	//! returns the next available problem
	/*!
	 * since by default the environment does not allow the test of all the problems, this methods does nothing
	 */
	virtual bool next_problem() { return false;};
};
#endif

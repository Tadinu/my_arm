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
// Filename      : woods2_env.hh
//
// Purpose       : implement the woods2 environment used by Wilson (1998)
//                 
// Special Notes : 
//                 
//
// Creator       : Pier Luca Lanzi
//
// Creation Date : 2002/08/05
//
// Current Owner : Pier Luca Lanzi
//-------------------------------------------------------------------------

/*!
 * \file woods2_env.hh
 *
 * \brief implements the woods environment as defined by Wilson (1995)
 *
 */


#ifndef __WOODS2_ENV__
#define __WOODS2_ENV__

#include <sstream>
#include "rl_definitions.hpp"
#include "environment_base.hpp"
#include "xcs_config_mgr2.hpp"

/*!
 * \class woods2_env woods2_env.hpp
 * \brief implements the methods specific for woods2 environments introduced in Wilson (1998)
 * \sa environment_base
 */

class woods2_env : public virtual environment_base
{
public:
	string class_name() const { return string("woods2_env"); };
	string tag_name() const { return string("environment::woods2"); };
	
	//! Constructor for the woods environment class. It reads the class parameters through the configuration manager.
	/*!
	 *  This is the only constructor that can be used. 
	 */
	woods2_env(xcs_config_mgr2&);

	//! Destructor for the woods environment class.
	~woods2_env();
	
	void begin_problem(const bool explore);
	void end_problem() { path << ends;};

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

	//! indicates that woods environments are multiple step problems
	virtual bool single_step() const {return false;};

 public:
	virtual double reward() const {assert(current_reward==woods2_env::current_reward); return current_reward;};
	virtual t_state state() const { return inputs; };
	virtual void print(ostream& output) const { output << "(" << current_pos_x << "," << current_pos_y << ")\t" << state();};

 private:
	//! computes the current position on an axis given the grid limits
	inline int	cicle(const int op, const int limit) const;

	//! computes the sensory inputs that are returned in position <x,y>
	void		get_input(const unsigned long x, const unsigned long y, t_state& sensors) const;

	//! given the current <x,y> position sets the current input and the current reward \sa current_position_x \sa current_position_y
	inline void	set_state();

	//! return true if the position <x,y> is free (i.e., it contains ".")
	inline bool	is_free(const unsigned long x, const unsigned long y) const;

	//! return true if the position <x,y> contains food (i.e., "F")
	inline bool	is_food(const unsigned long x, const unsigned long y) const;

	//! return true if the position <x,y> contains food (i.e., "F")
	inline void binary_encode(const string&, string&) const;
	
	static bool			init;			//!< true if the class has been inited through the configuration manager
	t_state			inputs;			//!< current input configuration

        //! true if the start position in the environment are set so to visit all the positions the same number of time
	bool		uniform_start;
	
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
	////unsigned long	current_configuration;			// counter for uniform problem start


	//! \var current_state current agent's input
	//unsigned long	current_state;				// counter for the scan of states

	//! \var configurations stores all the possible input configurations
	vector<string>	configurations;				// configurations

        //! current x position in the environment
	unsigned long 	current_pos_x;
        //! current y position in the environment
	unsigned long 	current_pos_y;

 	//! \var use_binary_sensors specifies whether binary inputs (i.e., 00100...) or symbolic inputs (i.e., "TF......T.") should be returned
	unsigned long   use_binary_sensors;

 	//! \var flag_binary_sensors specifies whether binary inputs (i.e., 00100...) or symbolic inputs (i.e., "TF......T.") should be returned
	bool		flag_binary_sensors;

        //! \var prob_slide specifies the probability that the agent can slip while it moves (Colombetti and Lanzi 1999)
	double		prob_slide; 

        //! \var env_rows number of rows
	unsigned long		env_rows;

        //! \var env_columns number of columns
	unsigned long		env_columns;

        //! \var map environment map 
	vector<string>		map;

        //! \var env_free_pos number of free positions (i.e., ".") in the environment
	unsigned long 		env_free_pos;

        //! x coordinates of the free positions in the environment
	vector<unsigned long>	free_pos_x;
	
        //! y coordinates of the free positions in the environment
	vector<unsigned long>	free_pos_y;

	//! path traces the path the agent followed during the problem
	ostringstream		path;
};
#endif

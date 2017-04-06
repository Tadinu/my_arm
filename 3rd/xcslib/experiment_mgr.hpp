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
// Filename      : experiment_mgr.hpp
//
// Purpose       : definition of the experiment manager class 
//                 
// Special Notes : 
//                 
//
// Creator       : Pier Luca Lanzi
//
// Creation Date : 2002/05/18
//
// Current Owner : Pier Luca Lanzi
//-------------------------------------------------------------------------

/*!
 * \file experiment_mgr.hpp
 *
 * \author Pier Luca Lanzi
 *
 * \version 0.01
 *
 * \date 2002/05/18
 * 
 * \brief defines the methods for the experiment manager 
 *
 */


#include "xcs_definitions.hpp"
#include "xcs_random.hpp"
#include "xcs_config_mgr2.hpp"

#ifndef __EXPERIMENT_MGR__
#define __EXPERIMENT_MGR__

/*!
 * \class experiment_mgr experiment_mgr.hpp
 *
 * \brief implements the experiment manager 
 *
 */

class experiment_mgr
{

public:
	//================================================================================
	//
	//
	//	PUBLIC METHODS
	//
	//
	//================================================================================
	
	//! name of the class that implements the experiment manager
	string class_name() const { return string("experiment_mgr"); };

	//! tag used to access the configuration file
	string tag_name() const { return string("experiments"); };

	//! class constructor; it reads the class parameters through the configuration manager
	experiment_mgr(xcs_config_mgr2 &xcs_config);

	//! perform the experiments
	void perform_experiments();

	//! print the flags for save various experiment statistics
	void print_save_options(ostream& output) const;

	//! save the state of the experiment
	void save_state(const unsigned long, const bool, unsigned long problem_no=0) const;
	
	//! restore the state of the experiment
	bool restore_state(const unsigned long);

private:
	//================================================================================
	//
	//
	//	PRIVATE VARIABLES
	//
	//
	//================================================================================
	
	long	current_experiment;		//!< the experiment currently running
	long	first_experiment;		//!< number of the first experiment to run
	long	no_experiments;			//!< numbero of total experiments to run

	long	current_problem;		//!< the problem being currently executed
	long	first_learning_problem;		//!< first problem executed
	long	no_learning_problems;		//!< number of problems executed
	long	no_condensation_problems;	//!< number of problems executed in condensation
	long	no_test_problems;		//!< number of test problems executed at the end
	long	no_max_steps;		//!< maximum number of step per problem
	long	do_trace;


	long	current_no_test_problems;	//!< number of test problems solved so far


	bool	  	do_test_environment;		//!< test the environment for every possible start

	long		save_interval;			//! the experiment status is saved every "save_interval" problems

	bool		flag_trace;			//!< true if the experiment outputs on the trace file
	bool		flag_test_environment;		//!< true if the system will be tested on the whole environment
	bool		flag_save_state;		//!< true if the state of the system will be saved at the end of the experiment
	bool		flag_save_agent_state;		//!< true if the state of the agent must be saved when an experiment ends
	bool		flag_save_agent_report;		//!< true if the state of the agent must be saved when an experiment ends
	bool		flag_trace_time;		//!< true if execution time is traced

	string		extension;			//!< file extension for the experiment files
	
	bool		flag_compact_mode;		//!< false if the statistics of every problem is saved
	unsigned long	save_stats_every;		//!< number of problems on which the average is computed and the statistics is reported
	double		compact_average_steps;
	double		compact_average_reward_sum;
	double		compact_average_size;

	bool		flag_buffered_output;		//! true if the outputs files are written only once at the end of the experiment

	//================================================================================
	//
	//
	//	PRIVATE METHODS
	//
	//
	//================================================================================
	
 private:
	//! save the agent state for experiment \emph expNo
	void save_agent_report(const unsigned long expNo, const unsigned long problem_no=0) const;

	//! save the agent state for experiment \emph expNo
	void save_agent_state(const unsigned long expNo, const unsigned long problem_no=0) const;
	
	//! restore the agent state for experiment \emph expNo
	void restore_agent(const unsigned long expNo) const;

};
#endif

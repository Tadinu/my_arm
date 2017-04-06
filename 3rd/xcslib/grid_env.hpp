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




//-------------------------------------------------------------------------
// Filename      : grid_env.hpp
//
// Purpose       : implements the 2d Gridworld
//                 
// Special Notes : 
//                 
//
// Creator       : Daniele Loiacono & Pier Luca Lanzi
//
// Creation Date : 2008/08/05
//
// Modifications : 
//
// Current Owner : Daniele Loiacono & Pier Luca Lanzi
//-------------------------------------------------------------------------

/*!
 * \file grid_env.hpp
 *
 * \brief implements the 2d Gridworld
 *
 */

#ifndef __GRID_ENV__
#define __GRID_ENV__

#include <sstream>
#include "rl_definitions.hpp"
#include "environment_base.hpp"
#include "xcs_config_mgr2.hpp"

/*!
 * \class grid_env grid_env.hpp
 * \brief implements the 2d Gridworld
 * \sa environment_base
 */

class grid_env : public virtual environment_base
{	

	//! define an obstacle as a rectangular area with an additional negative reward	
	class obstacle 
	{	
		public:
			
		//! coordinates of the top left corner of the obstacles
		double x1;
		double y1;
		
		//! coordinates of the bottom right corner of the obstacles
		double x2;
		double y2;
		
		//! additional cost of passing through obstacles
		double additional_cost;
		
		friend ostream& operator<<(ostream& output, const obstacle& o)
		{
			output << o.x1 << "\t" << o.y1 << "\t" << o.x2 << "\t" << o.y2;
			output << "\t" << o.additional_cost << endl;
		}
		
		friend istream& operator>>(istream& input, obstacle& o)
		{
			input >> o.x1 >> o.y1 >> o.x2 >> o.y2 >> o.additional_cost;
		}
	};
					
public:
	string class_name() const { return string("grid_env"); };

	string tag_name() const { return string("environment::gridworld"); };
	
	//! Class constructor that reads the class parameters through the configuration manager.
	grid_env(xcs_config_mgr2&);

	//! Class destructor.
	~grid_env();
	
	void begin_problem(const bool explore);
	void end_problem() {};

	void begin_experiment() {};
	void end_experiment() {};

	//! the problem ends when the top corner has been reached
	bool stop() const {return flag_corner_reached;};
	
	void perform(const t_action& action);

	void trace(ostream& output) const;

	bool allow_test() const {return true;};

	void reset_problem();
	bool next_problem();

	void reset_input();
	bool next_input();

	void save_state(ostream& output) const;
	void restore_state(istream& input);

	//! true if the problem is single-step or multi-step
	virtual bool single_step() const {return false;};

 	//! return the current reward
	double reward() const {assert(current_reward==grid_env::current_reward); return current_reward;};

	//! return the current state
	t_state state() const { return inputs; };

	//! print the current state to output
	void print(ostream& output) const { output << "(" << x << "," << y << ")\t" << state();};

 private:
	static bool			init;			//!< true if the class has been inited through the configuration manager
	t_state				inputs;			//!< current input configuration
 	vector<obstacle>		obstacles;		//!< obstacles of the environment

        //! true if the start position in the environment are set so to visit all the positions the same number of time
	bool		flag_fixed_start;
	
	//! current reward returned for the last action performed
	double		current_reward;

	//! \var step_size size of the action step 
	double 		step_size;

	//! parameters to scale and shift the input domain from the usual (0,0)-(1,1) square
	double 		shift;			//! term added to x and y (0 by default)
	double		scale;			//! term used to scale x and y (1 by default)
	
	//! sampling resolution used for testing
	double 		sampling_resolution;

    	//! current position
	double 		x;
	double 		y;
	
	//! problem x and y for testing
	double 		problem_x;
	double 		problem_y;
 
	//! path traces the path the car followed during the problem
	string		path;

	//! true if the goal corner has been reached
	bool		flag_corner_reached;

	void set_state() 
	{
		ostringstream str;
		str << x << " " << y;
		inputs.set_string_value(str.str());
	}

	//! load obstacles file	
	void load_obstacles(const string &fn);

	//! return the additional (negative) reward for position (x,y)
	double additional_reward (double x, double y);
};
#endif

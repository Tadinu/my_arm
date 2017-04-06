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
// Filename      : grid_env.cc
//
// Purpose       : implements and empty squared room [0,0] to [1,1]
//
// Special Notes :
//
//
// Creator       : Pier Luca Lanzi & Daniele Loiacono
//
// Creation Date : 2004/07/29
//
// Current Owner : Pier Luca Lanzi & Daniele Loiacono
//-------------------------------------------------------------------------

/*!
 * \file grid_env.cc
 *
 * \brief implements and empty squared room [0,0] to [1,1]
 *
 */
// #define __DEBUG_GRID_ENV__
#include "grid_env.hpp"

//! 	class not yet initialized
bool	grid_env::init = false;


grid_env::grid_env(xcs_config_mgr2& xcs_config)
{
	
	if (!grid_env::init)
	{		
		string		str_obstacles_fn;
		string		str_fixed_start;

		step_size = xcs_config.Value(tag_name(), "step size", 0.05);				//! step size is 0.05 by default
		str_fixed_start = (string) xcs_config.Value(tag_name(), "fixed start", "off");		//! start is random by default
		str_obstacles_fn = (string) xcs_config.Value(tag_name(), "obstacles file", "");	//! no obstacle by default
		sampling_resolution = xcs_config.Value(tag_name(), "sampling resolution", 0.05);	//! 

		scale = xcs_config.Value(tag_name(), "scale", 1.);				//! no scaling by default;
		shift = xcs_config.Value(tag_name(), "shift", 0.);				//! no shift by default;

		xcs_utility::set_flag(str_fixed_start, flag_fixed_start);
			
		if (str_obstacles_fn!="") load_obstacles(str_obstacles_fn);	
		
		grid_env::init = true;
		step_size *= scale;
		sampling_resolution *= scale;
	}
}


grid_env::~grid_env()
{
	//! nothing required
}

void
grid_env::begin_problem(const bool explore)
{
	ostringstream	PATH;

	//! at the beginning of the problem the previous information about the agent's path is cleared
	path = "";
	
	if (flag_fixed_start)
	{
		//! fixed start from (0,0)
		x=0;
		y=0;
	} else 	{
		//! start from a random position different from the goal
		do 
		{			
			x = xcs_random::random();
			y = xcs_random::random();			
		} while ((x>=1.0)&&(y>=1.0));
	}

	x = x*scale + shift;
	y = y*scale + shift;

#ifdef __DEBUG_GRID_ENV__
	cerr << "SCALE = " << scale << " SHIFT = " << shift << endl;
	cerr << "CURRENT POSITION = " << x << " " << y << endl;
#endif
	flag_corner_reached = false;

	current_reward = 0;

	set_state();

	PATH << state();
	path = "("+PATH.str()+")";

}

void
grid_env::perform(const t_action& action)
{	
	ostringstream 	PATH;
	static double	inc_x[] = { 0.0, 1.0,  0.0, -1.0};
	static double	inc_y[] = { 1.0, 0.0, -1.0,  0.0};

	double	next_x;
	double	next_y;

	unsigned long act = action.value();

	assert(act<4);

#ifdef __DEBUG_GRID_ENV__
	cout << "CURRENT = (" << x << " " << y << ") ==> ";
#endif

	next_x = x + inc_x[act]*step_size;
	next_y = y + inc_y[act]*step_size;
		
#ifdef __DEBUG_GRID_ENV__
	cout << "NEXT = (" << next_x << " " << next_y << ") ";
	cout << "GOAL = (" << scale+shift << " " << scale+shift << ") ";
#endif

	if ((next_x >= scale+shift) && (next_y >= scale+shift))
		flag_corner_reached = true;
		
	next_x = max(next_x, shift);
	next_x = min(next_x, shift+scale);
	
	next_y = max(next_y, shift);
	next_y = min(next_y, shift+scale);
	
	x = next_x;
	y = next_y;

#ifdef __DEBUG_GRID_ENV__
	cout << "TRUE = (" << next_x << " " << next_y << ")";
	cout << endl;

	cerr << "CURRENT POSITION = " << x << " " << y << endl;
#endif

	if (flag_corner_reached)
		current_reward = 0;
	else 
		current_reward = -0.5 + additional_reward((x-shift)/scale,(y-shift)/scale);		
	
	//cout << "CURRENT REWARD = " << current_reward << endl;

	char c1,c2;
	if (x==1)
		c1='*';
	else
		c1=' ';
	if (y==1)
		c2='*';
	else
		c2=' ';
	set_state();
	PATH << "(" << state() << c1 << c2 <<")";
	path = path +PATH.str();

}

void
grid_env::trace(ostream& output) const
{
	output << path;
}

void
grid_env::reset_input()
{
	problem_x = shift;
	problem_y = shift;

	x = shift;
	y = shift;
	set_state();
}

bool
grid_env::next_input()
{
	problem_y+=sampling_resolution;
	if (problem_y>=scale+shift+sampling_resolution)
	{
		problem_y = shift;
		problem_x+=sampling_resolution;
		if (problem_x>=shift+scale+sampling_resolution) return false;
	}

	x = problem_x;
	y = problem_y;

	set_state();

	return true;
}

void
grid_env::save_state(ostream& output) const
{
	output << x << '\t' << y << '\t';	
}

void
grid_env::restore_state(istream& input)
{
	input >> x >> y;	
}

void
grid_env::reset_problem()
{
	reset_input();
}


bool
grid_env::next_problem()
{
	return next_input();
}

void 
grid_env::load_obstacles(const string &fn)
{
	ifstream in(fn.c_str());
	if(!in.good())
	{
		xcs_utility::error(class_name(),"class constructor", "Cannot open obstacles file",1);
	}
	
	obstacle obst;
	obstacles.clear();
	while (in>>obst)
	{
		obstacles.push_back(obst);
		cerr << "One obstacle Loaded: " << obst << endl;
	}
}


double 
grid_env::additional_reward (double x, double y)
{
	double add_rew = 0;
	for (int i=0; i < obstacles.size(); i++)
	{
		if ( (obstacles[i].x1 <= x) && (obstacles[i].x2 >= x) 
				&& (obstacles[i].y1 <= y) && (obstacles[i].y2 >= y) )
		{
			add_rew += obstacles[i].additional_cost;
		}		
	}
	return add_rew;	
}

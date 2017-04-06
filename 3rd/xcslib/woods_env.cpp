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
// Filename      : woods_env.cc
//
// Purpose       : implement the woods environment used by Wilson (1995)
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
 * \file woods_env.cc
 *
 * \brief implementation of woods environments as defined by Wilson (1995)
 *
 * The environment is defined through a map contaning obstacles (T), free positions (.), and goal positions (F).
 * The top left position of the map corresponds to position (0,0).
 *
 * At the beginning of a problem the agent is place at a random position; 
 * the problem ends when the agent reaches a goal position.
 *
 * The configuration file specifies two main information: 
 * - the name of the file that contains the enviroment map and 
 * - the agent sliding probability 
 *
 */
#include "woods_env.hpp"

//! \def __WOODS_ENV_CFG_IN__ specifies the format of the configuration file for the woods environment
#define	__WOODS_ENV_CFG_IN__ "\
		map = %s \
		binary sensors = %s \
		slide probability = %lf "

//! \def __WOODS_ENV_VARS_IN__ specifies the parameters that are read from the configuration file 
/*!
 * \sa map_file
 * \sa use_binary_sensors
 * \sa uniform_start
 * \sa prob_slide
 */

#define __WOODS_ENV_VARS_IN__ map_file, &str_use_binary_sensors, &prob_slide

//! specifies the format string to print the current configuration parameters
#define	__WOODS_ENV_CFG_OUT__ " \n\
		Map = %s \n\
		BinarySensors = %s \n\
		SlideProbability = %lf "

//! specifies the variables read from the configuration file that are printed on screen for logging purposes
#define __WOODS_ENV_VARS_OUT__ map_file, str_use_binary_sensors, prob_slide


bool	woods_env::init = false;

woods_env::woods_env(xcs_config_mgr2& xcs_config)
{
	if (!woods_env::init)
	{
		string		map_file;
		string		str_use_binary_sensors;

		if (!xcs_config.exist(tag_name()))
		{
			xcs_utility::error(class_name(), "constructor", "section <" + tag_name() + "> not found", 1);	
		}
	
		try {
			map_file = (string) xcs_config.Value(tag_name(), "map");
			str_use_binary_sensors = (string) xcs_config.Value(tag_name(), "binary sensors");
			prob_slide = xcs_config.Value(tag_name(), "slide probability");
		} catch (const char *attribute) {
			string msg = "attribute \'" + string(attribute) + "\' not found in <" + tag_name() + ">";
			xcs_utility::error(class_name(), "constructor", msg, 1);
		}

		/// set the flag
		
		xcs_utility::set_flag(string(str_use_binary_sensors), flag_binary_sensors);

		///	read the woods map
		ifstream MAP(map_file.c_str());

		if (!MAP.good())
		{
			// error: map not found
			xcs_utility::error(class_name(),"class constructor", "map file not found", 1);
		}

		env_rows = 0;
		env_columns = 0;
		env_free_pos = 0;

		bool		first_row=true;
		string		row;
	
		map.clear();
		while (MAP >> row)
		{
			if (first_row)	
			{
				env_columns = row.size();
				first_row=false;
			}
			else if (row.size()!=env_columns)
			{
				xcs_utility::error(class_name(),"class constructor", "unrecognized map format", 1);
			}	
			map.push_back(row);
	
			string::iterator	pos;
	
			for(pos=row.begin(); pos!=row.end(); pos++)
			{
				if (*pos=='.')
				{
					env_free_pos++;
				}
			}
		}
		MAP.close();
	
		env_rows = map.size();

#ifdef __DEBUG
		clog << "ok." << endl;
		clog << "map is " << env_columns << "x" << env_rows << endl;

		clog << "file " << map_file << endl;
		clog << endl;
		
		for(unsigned long row=0; row<map.size(); row++)
		{
			clog << map[row] << endl;
		}
		clog << endl;
#endif

		///
		free_pos_x.clear();
		free_pos_y.clear();
	
		t_state		free_position;
		configurations.clear();

		for (unsigned long y=0; y<env_rows; y++)
		{
			for (unsigned long x=0; x<env_columns; x++)
			{
				if (is_free(x,y))
				{
					get_input(x,y,free_position);
					configurations.push_back(free_position.string_value());

					free_pos_x.push_back(x);
					free_pos_y.push_back(y);
				}
			}
		}
	
	
		current_pos_x = 0;
		current_pos_y = 0;
	
		current_configuration = 0;
	
		set_state();

	}
	woods_env::init = true;
}
woods_env::~woods_env()
{
	map.clear();
}

void	
woods_env::begin_problem(const bool explore)
{
	ostringstream	PATH;
   	unsigned long	where;

	//! at the beginning of the problem the previous information about the agent's path is cleared
	path = "";

	//! random restart
	where = (unsigned long) (env_free_pos*xcs_random::random());
		
	current_pos_x = free_pos_x[where];
	current_pos_y = free_pos_y[where];
	set_state();

	PATH << "(" << current_pos_x << "," << current_pos_y << ")";
	path += PATH.str();
}

bool	
woods_env::stop()
const
{
	return(is_food(current_pos_x,current_pos_y)); 
}


void	
woods_env::perform(const t_action& action)
{
	static int 	sliding[] = {-1,1};
	static int	inc_x[] = { 0, +1, +1, +1,  0, -1, -1, -1};
	static int	inc_y[] = {-1, -1,  0, +1, +1, +1,  0, -1};

	unsigned long	act;
	unsigned long	next_x;
	unsigned long	next_y;

	act = action.value();

	if (xcs_random::random()<prob_slide)
	{
		act = cicle(act + sliding[xcs_random::dice(2)], action.actions());
	}

	next_x=cicle(current_pos_x+inc_x[act], env_columns);
	next_y=cicle(current_pos_y+inc_y[act], env_rows);

	if ((is_free(next_x,next_y)) || (is_food(next_x,next_y)))
	{
		current_pos_x = next_x;
		current_pos_y = next_y;
		set_state();
	}
	ostringstream PATH;
	PATH << "(" << current_pos_x << "," << current_pos_y << ")";
	path += PATH.str();
}

void
woods_env::trace(ostream& output) const
{
	output << path;
}

void 
woods_env::reset_input()
{
	reset_problem();
}

bool 
woods_env::next_input()
{
	return next_problem();
}

void
woods_env::save_state(ostream& output) const
{
	output << endl;
	output << current_pos_x << '\t' << current_pos_y << '\t';
	output << current_configuration << endl;
}

void
woods_env::restore_state(istream& input)
{
	input >> current_pos_x; 
	input >> current_pos_y;
	input >> current_configuration;
	set_state();
}

inline 
int	
woods_env::cicle(const int op, const int limit) 
const
{
	return ((op % limit)>=0)?(op % limit):((op % limit) + limit);
}

inline
void	
woods_env::get_input(const unsigned long x,const unsigned long y, t_state& inputs)
const
{
	static int	inc_x[] = { 0, +1, +1, +1,  0, -1, -1, -1};
	static int	inc_y[] = {-1, -1,  0, +1, +1, +1,  0, -1};
	unsigned long	sx,sy;
	unsigned long	pos;

	string		symbolic_input = "";
	string		binary_input = "";

	for(pos=0; pos<8; pos++)
	{
		sx = cicle(x+inc_x[pos], env_columns);
		sy = cicle(y+inc_y[pos], env_rows);
		symbolic_input += map[sy][sx];
	}

	if (!flag_binary_sensors)
	{
		inputs.set_string_value(symbolic_input);
	} else {
		string	binary_input;
		binary_encode(symbolic_input, binary_input);
		inputs.set_string_value(binary_input);
	}
}

inline
void	
woods_env::set_state()
{
	get_input(current_pos_x,current_pos_y,inputs);

	if (is_food(current_pos_x,current_pos_y))
	{
		current_reward = 1000;
	} else {
		current_reward = 0;
	}
}

inline
bool 
woods_env::is_free(const unsigned long x, const unsigned long y)
const
{
	return(map[y][x]=='.');
}

inline
bool 
woods_env::is_food(const unsigned long x, const unsigned long y)
const
{
	return (map[y][x]=='F');
}

void
woods_env::binary_encode(const string &input, string &binary)
const
{
	string::const_iterator	pos;

	binary = "";
	pos=input.begin();

	for(pos=input.begin(); pos!=input.end(); pos++)
	{
		switch (*pos)
		{
			case 'T': // tree
				binary += "10";
				break;
			case 'F': // food
				binary += "11";
				break;
			case '.': // free 
				binary += "00";
				break;
			default:
				xcs_utility::error(class_name(),"class constructor", "unrecognized map symbol", 1);
		}
	}
}

void
woods_env::reset_problem()
{
	path = "";

	current_configuration = 0;
	current_pos_x = free_pos_x[current_configuration];
	current_pos_y = free_pos_y[current_configuration];
	set_state();
	
	ostringstream PATH;
	PATH << "(" << current_pos_x << "," << current_pos_y << ")";
	path = PATH.str();
}


bool
woods_env::next_problem()
{
	current_configuration++;
	if (current_configuration==env_free_pos)
	{
		reset_problem();
		return false;
	}

	current_pos_x = free_pos_x[current_configuration];
	current_pos_y = free_pos_y[current_configuration];
	set_state();
	ostringstream PATH;
	PATH << "(" << current_pos_x << "," << current_pos_y << ")";
	path = PATH.str();
	return true;
}

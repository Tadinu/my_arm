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
// Filename      : xcs_random.cc
//
// Purpose       : implements the random number generator for XCS
//                 
// Special Notes : 
//                 
//
// Creator       : Pier Luca Lanzi
//
// Creation Date : 2002/05/14
//
// Current Owner : Pier Luca Lanzi
//-------------------------------------------------------------------------
// Updates
//		   2003 11 14 added the seed option. if seed=0 time is used
//-------------------------------------------------------------------------

/*!
 * \file xcs_random.cc
 *
 * \brief implements the random number generator for XCS
 *
 * \author Pier Luca Lanzi
 *
 * \version 0.01
 *
 * \date 2002/05/27
 * 
 */

#include <cmath>
#include <ctime>
#include <cstdlib>
#include <iostream>
#include "xcs_random.hpp"

using namespace std;

//! \var unsigned long xcs_random::seed is seed for random number generator.
unsigned long	xcs_random::seed = 0;

/*! 
 * \fn void xcs_random::set_seed(xcs_config_mgr& xcs_config)
 * \param xcs_config represents the configuration manager currently in use.
 *
 * \brief set the seed for random number generation through the configuration manager.
 */
void
xcs_random::set_seed(xcs_config_mgr2& xcs_config)
{

	//! look for the init section in the configuration file
	if (!xcs_config.exist(tag_name()))
	{
		xcs_utility::error(class_name(), "constructor", "section <" + tag_name() + "> not found", 1);	
	}
	
	xcs_config.save(cerr);
	
	try {
		seed = xcs_config.Value(tag_name(), "seed");
	} catch (const char *attribute) {
		string msg = "attribute \'" + string(attribute) + "\' not found in <" + tag_name() + ">";
		xcs_utility::error(class_name(), "constructor", msg, 1);
	}

	if (xcs_random::seed!=0)
	{
		xcs_random::set_seed(xcs_random::seed);
	} else {
		//! use clock to set the seed
		srand48(time(NULL));
	}
}

/*! 
 * \fn float xcs_random::random()
 *
 * \brief generate a real random number between 0 and 1.
 */
double 
xcs_random::random()
{
#ifndef __XCS_MT_RANDOM__
	return (double) drand48();
#else

#endif
}

void 
xcs_random::set_seed(long new_seed)
{
#ifndef __XCS_MT_RANDOM__
	xcs_random::seed = new_seed;
 	srand48(xcs_random::seed);
#else

#endif
}

/*! 
 * \fn unsigned int xcs_random::dice(const unsigned int limit)
 *
 * \brief generate an integer random number between 0 and limit - 1.
 */
unsigned int 
xcs_random::dice(const unsigned int limit)
{
	return ((unsigned int)((xcs_random::random()*float(limit))));
}

/*!
 * \fn double nrandom()
 *
 * \brief returns a floating-point random number generated according to a normal distribution with mean 0 and standard deviation 1
 */

double
xcs_random::nrandom()
{
	double x1, x2, w;
	static bool	flag_have_number=false;		//! true if a number has been already generated
	static double	generated_number;		//! number generated at previous step

	if(flag_have_number)
	{
		flag_have_number = false; 
		return generated_number;
	} else {
		do
		{
			x1 = 2.0 * xcs_random::random() - 1.0;
			x2 = 2.0 * xcs_random::random() - 1.0;
			w = x1 * x1 + x2 * x2;
		} while ( w >= 1.0 );
   	 
		w = sqrt( (-2.0 * log( w ) ) / w );
		generated_number = x1 * w;
		flag_have_number = true;
		return x2 * w;
	}
  
}

//! returns a random sign
int
xcs_random::sign()
{
	if (xcs_random::random()<0.5)
		return -1;
	else
		return 1;
}

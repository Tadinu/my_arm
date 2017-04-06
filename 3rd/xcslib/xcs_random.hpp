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
// Filename      : xcs_random.hh
//
// Purpose       : defines the random number generator for XCS
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
// Modification  : 2003/05/07 added the generation of random number
//                            according to Gaussian distribution
//-------------------------------------------------------------------------

/*!
 * \file xcs_random.hh
 *
 * \brief class definition for the XCS random number generator
 *
 * \author Pier Luca Lanzi
 *
 * \version 0.01
 *
 * \date 2002/05/27
 * 
 */


#ifndef __XCS_RANDOM__
#define __XCS_RANDOM__

#include "xcs_config_mgr2.hpp"

/*!
 * \class xcs_random
 *
 * \brief implements the random number generator for XCS
 *
 * \author Pier Luca Lanzi
 *
 * \version 0.01
 *
 * \date 2002/05/15
 *
 */

class xcs_random {
 private:
	//!  \var unsigned long seed to initialize the random number generator
	static unsigned long seed;
 public:
	//! name of the class that implements the random number generator.
	static string class_name() { return string("xcs_random"); };
	
	//! tag used to access the configuration file
	static string tag_name() { return string("random"); };

	//! set the seed for random number generation through the configuration manager
	static void set_seed(xcs_config_mgr2&);

	//! set the seed from random number generation
	static void set_seed(long seed);

	//! returns a random number in the real interval [0,1).
	static double random();

	//! returns a random number from a Gaussian distribution 
	static double nrandom();

	//! returns a random sign
	static int sign();

	/*! \brief returns an integer random number between 0 and limit-1.
	 *  \param limit the upper boundary for number generation
	 */
	static unsigned int dice(unsigned int limit);

	//! Saves the state of the random number generator to an output stream.
	/*! 
	 * It currently does not perform any action.
	 */
	static void save_state(ostream& output) {};

	//! Restores the state of the random number generator from an input stream.
	/*! 
	 * It currently does not perform any action.
	 */
	static void restore_state(istream& input) {};
};
#endif

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
// Filename      : xcs_definitions.hh
//
// Purpose       : mapping between the high level class names and the 
//                 specific classes used for implementing XCS
//                 defining types, classes, and constants for all the files.
//                 
// Special Notes : 
//                 
//
// Creator       : Pier Luca Lanzi
//
// Creation Date : 2002/06/10
//
// Current Owner : Pier Luca Lanzi
//-------------------------------------------------------------------------



/*!
 * \file xcs_definitions.hh
 *
 * \brief maps high level class names (e.g., t_condition) to the actual class name (e.g., xcs_bitstring_condition)
 *
 */

#define __XCSLIB_VERSION__ "1.1"

#ifndef __XCS_DEFINITIONS__
#define __XCS_DEFINITIONS__
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string.h>
#include <iomanip>

// STL libraries
#include <algorithm>
#include <vector>

#include "rl_definitions.hpp"

#if defined XCS_LIB
// maps the actual class used for classifier conditions, specified with the __CONDITION__ variable in 
// the make file to the high level name t_condition
class   ternary_condition;
typedef ternary_condition t_condition;

#include "ternary_condition.hpp" //__COND_INCLUDE__



//! maps the actual class used for the environment, specified with the __ENVIRONMENT__ variable in 
//! the make file to the high level name t_environment
#include "xcs_classifier.hpp" // __CLS_INCLUDE__

class   xcs_classifier;
typedef xcs_classifier t_classifier;



//! maps the actual class used for the the classifier system, specified with the __MODEL__ variable in 
//! the make file to the high level name t_environment
#include "xcs_classifier_system.hpp"
class   xcs_classifier_system;
typedef xcs_classifier_system t_classifier_system;
#elif defined XCSF_LIB
// maps the actual class used for classifier conditions, specified with the __CONDITION__ variable in
// the make file to the high level name t_condition
class   real_interval_condition;
typedef real_interval_condition t_condition;

#include "real_interval_condition.hpp" //__COND_INCLUDE__



//! maps the actual class used for the environment, specified with the __ENVIRONMENT__ variable in
//! the make file to the high level name t_environment
#include "xcsf_classifier.hpp" // __CLS_INCLUDE__

class   xcsf_classifier;
typedef xcsf_classifier t_classifier;



//! maps the actual class used for the the classifier system, specified with the __MODEL__ variable in
//! the make file to the high level name t_environment
#include "xcsf_classifier_system.hpp"
class   xcsf_classifier_system;
typedef xcsf_classifier_system t_classifier_system;
#endif
#endif

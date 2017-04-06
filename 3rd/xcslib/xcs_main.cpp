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
// Filename      : main.cc
//
// Purpose       : main
//                 
// Special Notes : 
//                 
//
// Creator       : Pier Luca Lanzi
//
// Creation Date : 2002/05/28
//
// Current Owner : Pier Luca Lanzi
//-------------------------------------------------------------------------
// Modifications
// 20050429	added the verbose flag
// 20030807	general.hh becomes xcs_definitions.hh
//-------------------------------------------------------------------------

/*!
 * \file main.cc
 *
 * \brief implements the main code to exploit the 
 *
 */

#include "xcs_definitions.hpp" 
#include "experiment_mgr.hpp"

/*!
//	global variables
*/

/*!
 * \var t_environment* Environment
 * the environment
 */

/*!
 * \var xcs_classifier_system* XCS
 * the XCS classifier system
 */

/*!
 * \var experiment_mgr* Session
 * the experiment session
 *
 */

t_environment		*Environment;
t_classifier_system	*XCS;

experiment_mgr* Session;
bool flag_verbose = false;		//! if true verbose output is printed

/*! 
 * \fn int main(int Argc, char *Argv[])
 * \param argc number of arguments
 * \param argv list of arguments
 * 
 * main code to exploit the XCS implementation
 */
int
main(int argc, char *argv[])
{
	string	str_suffix = "maze"; 		//! configuration file suffix
	int		o;							//! current option
	
	if (argc==1)
	{
		cerr << "USAGE:\t\t" << argv[0] << "\t" << "-f <suffix> [-v] [-s <set>] " << endl;
		cerr << "      \t\t\t\t" << "<suffix>     suffix for the configuration file" << endl;
		cerr << "      \t\t\t\t" << "-v           verbose output" << endl;
		cerr << "      \t\t\t\t" << "-h           print version" << endl;
		return 0;
	}		


	while ( (o = getopt(argc, argv, "hvf:")) != -1 )
	{
		switch (o)
		{
			case 'v':
				flag_verbose = true;
			
			case 'f':
				str_suffix = string(optarg);
				break;
			case 'h':
				//! print version
				cerr << "XCSLIB\tVERSION " << __XCSLIB_VERSION__ << endl;
				cerr << "      \tBUILT   " << __DATE__ << endl;
				cerr << "      \tTIME    " << __TIME__ << endl;
				cerr << endl;
				cerr << "      \tSTATE       " << __INPUTS_VERSION__ << endl;
				cerr << "      \tACTION      " << __ACTION_VERSION__ << endl;
				cerr << "      \tCONDITIIONS " << __CONDITION_VERSION__ << endl;
				cerr << endl << endl;
				return 0;
				break;	
			default:
				string msg = "unrecognized option" + string(optarg);
				xcs_utility::error("main","main",msg,0);
		}
	}
	

	//! system configuration begins
	if (flag_verbose) 
		cout << "\nSystem Start ..." << endl;

	string			suffix(str_suffix);
	
	//! init the configuration manager
	xcs_config_mgr2	xcs_config2(suffix);
	if (flag_verbose) 
		cout << "Configuration Manager\t\tok." << endl;


	//! init random the number generator
	xcs_random::set_seed(xcs_config2);
	if (flag_verbose) 
		cout << "Random numbers         \t\tok." << endl;

	//! init the action class
	t_action		dummy_action(xcs_config2);
	if (flag_verbose) 
		cout << "Actions                \t\tok." << endl;

	//! init the environment
	Environment = new t_environment(xcs_config2);
	if (flag_verbose) 
		cout << "Environment            \t\tok." << endl;

	//! init the condition class
	t_condition		dummy_condition(xcs_config2);
	if (flag_verbose) 
		cout << "Conditions             \t\tok." << endl;

	//! init the XCS classifier system
	XCS = new t_classifier_system(xcs_config2);
	if (flag_verbose) 
		cout << "Classifier System      \t\tok." << endl;

	//! init the experiment manager
	Session = new experiment_mgr(xcs_config2);
	if (flag_verbose) 
	{
		cout << "Session Manager        \t\tok." << endl;
		cout << endl;
	}

	//! the experiment session begins
	if (flag_verbose) 
		cout << "Begin Experiments...\n" << endl;
	Session->perform_experiments();
	if (flag_verbose) 
		cout << "End Experiments...\n" << endl;
	
}

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
// Filename      : xcs_utility.hh
//
// Purpose       : utilities for XCS implementation
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
// Updates
//	2005 07 27 added trim, string2flag, and flag2string
//-------------------------------------------------------------------------

/*!
 * \file xcs_utility.hh
 *
 * \brief implements various utility functions for XCS implementation
 *
 */

/*!
 * \namespace xcs_utility
 *
 * \brief functions which are needed in various parts of XCS code.
 *
 * This namespace collect the common functions used in different part of 
 * XCS code. It includes functions for binary to integer conversion, integer to binary
 * conversion, parameter settings, etc.
 *
 * \author Pier Luca Lanzi
 *
 * \version 0.01
 *
 * \date 2002/05/28
 *
 */

#include <sys/times.h>
#include <ctime>
#include <iomanip>
#include <unistd.h>
#include <string>
#include <vector>
#include <algorithm>

using namespace std;

#ifndef __XCS_UTILITY__
#define __XCS_UTILITY__

namespace xcs_utility {

	//! error codes
	typedef enum {
		//! error while reading the configuration file
		ERR_CONFIGURATION_FILE = 0, 
	
	} t_error_code;

	//! error messages associated to codes \sa t_error_code
	const string error_message[] = {"Unrecognized configuration file"};

	//! convert a string that represents a binary number to a unsigned long
	unsigned long binary2long(const string&);

	//! convert an unsigned long to a binary string
	string long2binary(const unsigned long, unsigned long);

	//! conver a number to string filling up sz positions;
	string number2string(unsigned long n, unsigned long sz);

	//! set the Boolean variable to true if the string is "on"; to false if the string is "off"
	void set_flag(string set, bool& var);

	bool string2flag(const string str);
	string flag2string(bool f);

	//! print an error message specified with a string 
	void error(string name, string method, string message, unsigned int exit_code);
	
	//! print a warning message specified with a string 
	void warning(string name, string method, string message);
	
	//! print an error message specified with a a code \sa t_error_code
	void error(string name, string method, t_error_code message, unsigned int exit_code);

	string trim(string const& source, char const* delims = " \t\r\n");
};

/**
 * \class number_set
 *
 * \brief (deprecated) defines a set of integers which is used to select covering sensors for GP
 *
 * defines a set of integers which is used to select covering sensors for GP
 *
 */

class number_set
{
  private:
	  	//! \var numbers represent the set
		vector<unsigned long> numbers;
  public:
		//! create a set variable containing the numbers from 0 to n-1
		number_set(const unsigned long n);

		//! remove an integer from the set
		void remove(const unsigned long n);

		//! extract a number from the set, the number is deleted; return -1 if there the set is empty
		long extract();

		//! return the size of the set
		unsigned int size() const {return numbers.size();};

		//! init the set with the numbers from 0 to n-1
		void reset(const unsigned long n);
		 
};


//! timer class to measure CPU execution time
class timer {
 private:
	 unsigned long ti;	//! init time
	 unsigned long tf;	//! stop time

 public:
	 timer() {
		struct tms reading;
		times(&reading);
		ti = reading.tms_utime;
	 };

	 ~timer() {};
	 void start()
	 {	 
		struct tms reading;
		times(&reading);
		ti = reading.tms_utime;
	 }

	 double time() const
	 {
		struct tms reading;
		times(&reading);
		return double(reading.tms_utime - ti)/sysconf(_SC_CLK_TCK);
	 }

	 void stop()
	 {	 
		struct tms reading;
		times(&reading);
		tf = reading.tms_utime;
	 }

	 double elapsed() const
	 {
		return double(tf - ti)/sysconf(_SC_CLK_TCK);
	 }

	 unsigned long initial() const { return ti; };
	 unsigned long final() const { return tf; };
};
#endif

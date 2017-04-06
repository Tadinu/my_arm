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
// Filename      : xcs_utility.cc
//
// Purpose       : implementation of XCS utilities 
//                 
// Special Notes : 
//                 
//
// Creator       : Pier Luca Lanzi
//
// Creation Date : 2002/05/13
//
// Current Owner : Pier Luca Lanzi
//
// Updates
//                 2005 07 18 added string2flag and flag2string
//-------------------------------------------------------------------------

/*!
 * \file xcs_utility.cc
 *
 * \brief implements various utility functions for XCS implementation
 *
 */

#include <iostream>
#include <sstream>
#include <cstdlib>
#include "xcs_utility.hpp"

using namespace std;

unsigned long 
xcs_utility::binary2long(const string &binary)
{
	unsigned long		power = 1;
	unsigned long		integer = 0;
	string::size_type	bit;

	for(bit=0; bit<binary.size(); bit++)
	{
		integer += (binary[binary.size()-bit-1]-'0') * power;
		power *= 2;
	}
	return integer;
}

string 
xcs_utility::long2binary(const unsigned long decimal, unsigned long size)
{
	string::size_type	bit;
	string			binary;
	unsigned long		base = 1;

	base <<= size;
	for (bit=0; bit<size; bit++)
	{
		base >>= 1;
		binary += '0'+((decimal & base)>0);
	};
	return binary;
}

void 
xcs_utility::set_flag(string set, bool& var)
{
	if ((set=="on")||(set=="ON"))
		var = true;
	else if ((set=="off")||(set=="OFF"))
		var = false;
	else {
		xcs_utility::error("xcs_utility", "set_flag", "value must be 'on' or 'off'", 1);
	}
}

bool
xcs_utility::string2flag(const string str)
{
	if (str=="on")
		return true;
	else if (str=="off")
		return false;
	else
		xcs_utility::error("xcs_utility", "string2flag", "value must be 'on' or 'off'", 1);
}

string
xcs_utility::flag2string(bool f)
{
	if (f) 
		return "on";
	else 
		return "off";
}

void 
xcs_utility::error(string name, string method, string message, unsigned int exit_code)
{
	cerr << "\n\a***\tERROR";
	cerr <<   "\n***\tCLASS:  \t" << name;
	cerr <<   "\n***\tMETHOD: \t" << method;
	cerr <<   "\n***\tMESSAGE:\t" << message << "." << endl << endl;
	if (exit_code>0)
	{
		exit(exit_code);
	}
}

void 
xcs_utility::warning(string name, string method, string message)
{
	cerr << "\n\a***\tWARNING";
	cerr <<   "\n***\tCLASS:  \t" << name;
	cerr <<   "\n***\tMETHOD: \t" << method;
	cerr <<   "\n***\tMESSAGE:\t" << message << "." << endl << endl;
}

void 
xcs_utility::error(string name, string method, t_error_code message, unsigned int exit_code)
{
	cerr << "\n\a***\tERROR    \t";
	cerr <<   "\n***\tCLASS:   \t" << name;
	cerr <<   "\n***\tMETHOD:  \t" << method;
	cerr <<   "\n***\tMESSAGE: \t" << xcs_utility::error_message[message] << "." << endl << endl;
	if (exit_code>0)
	{
		exit(exit_code);
	}
}

string 
xcs_utility::number2string(unsigned long n, unsigned long sz)
{
	ostringstream RESULT;
	RESULT << n;

	string result = RESULT.str();

	for(unsigned long dgt=result.size(); dgt<sz; dgt++)
	{
		result = '0' + result;
	}
	return result;
}

//! delete leading and trailing spaces
string
xcs_utility::trim(string const& source, char const* delims)
{
	string result(source);
	string::size_type index = result.find_last_not_of(delims);

	if(index != std::string::npos)
	{
		result.erase(++index);
	}

	index = result.find_first_not_of(delims);
	if(index != std::string::npos)
		result.erase(0, index);
	else
		result.erase();
	return result;
}

number_set::number_set(const unsigned long n)
{
	reset(n);
}

long
number_set::extract()
{
	unsigned long num;

	if (!numbers.size())
		return -1;

	num = numbers.back();

	numbers.pop_back();

	return num;

}

void number_set::reset(const unsigned long n)
{
	numbers.reserve(n);

	for (unsigned long num=0; num<n; num++)
	{
		numbers.push_back(num);
	};

	random_shuffle(numbers.begin(),numbers.end());
}

void number_set::remove(const unsigned long n)
{
	vector<unsigned long>::iterator	pos;

	pos = find(numbers.begin(), numbers.end(), n);

	if (pos != numbers.end())
		numbers.erase(pos);
}


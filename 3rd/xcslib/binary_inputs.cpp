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



#include <string>
#include "binary_inputs.hpp"

void 
binary_inputs::set_string_value(const string &str)
{
#ifdef __XCS_DEBUG__
	string::size_type	bit;

	bit = 0;
	
	while ((bit<str.size()) && ((str[bit]=='0') || (str[bit]=='1')))
	{
		bit++;
	}

	if (bit!=str.size())
	{	
		xcs_utility::error(class_name(),"set_string_value", "wrong size in assignement.", 1);
	}
#endif
	value = str;
};

char 
binary_inputs::input(unsigned long pos)
const
{
	assert(pos<size());
	return value[pos];
}

void
binary_inputs::set_input(unsigned long pos, char val)
{
	assert(pos<size());
	assert( (val=='0') || (val=='1') );
	value[pos] = val;
}

void
binary_inputs::numeric_representation(vector<long>& nr)
const
{
	string::const_iterator ch;

	nr.clear();

	for(ch=value.begin(); ch!=value.end(); ch++)
	{
		nr.push_back(long(*ch-'0'));
	}
}

void
binary_inputs::set_numeric_representation(const vector<long>& nr)
{
	vector<long>::const_iterator	val;
	value = "";

	for(val=nr.begin(); val!=nr.end(); val++)
	{
		assert((*val==1)&&(*val==0));
		value += char(*val+'0');
	}
}

void
binary_inputs::numeric_representation(vector<double>& nr)
const
{
#ifndef __ZERO_AS_NEGATIVE__

	string::const_iterator ch;

	nr.clear();

	for(ch=value.begin(); ch!=value.end(); ch++)
	{
		nr.push_back(double(*ch-'0'));
	}
#else
#define V 1

  string::const_iterator ch;

  nr.clear();

  for(ch=value.begin(); ch!=value.end(); ch++)
  {
	//double VV = 1/double(value.size());
	double VV = 5; 
        if (*ch=='0')
          nr.push_back(-VV);
        else
          nr.push_back(VV);
  }

#endif
}

void
binary_inputs::set_numeric_representation(const vector<double>& nr)
{
	vector<double>::const_iterator	val;
	value = "";

	for(val=nr.begin(); val!=nr.end(); val++)
	{
		assert((*val==1)&&(*val==0));
		value += char(*val+'0');
	}
}

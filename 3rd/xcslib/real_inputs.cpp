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




#include <string>
#include <sstream>
#include <cassert>
#include <vector>

#include "real_inputs.hpp"

real_inputs::real_inputs(const real_inputs& rs)
{
        this->no_inputs = rs.no_inputs;
        this->inputs = rs.inputs;
}

void
real_inputs::set_string_value(const string &str)
{	
	inputs.clear();
	istringstream in(str);
	double val;
	long dim=0;

	while (in>>val)
	{
		inputs.push_back(val);
		dim++;
	}
	no_inputs = dim;
}

string
real_inputs::string_value() const
{
	ostringstream out;
	out << inputs[0];
	for (int i=1; i<no_inputs; i++)
	{
		out << ' ' << inputs[i];
	}
	return out.str();
}

double
real_inputs::input(unsigned long pos)
const
{
        assert(pos<no_inputs);
        return inputs[pos];
}

void
real_inputs::set_input(unsigned long pos, double val)
{
        assert(pos<no_inputs);
        inputs[pos] = val;
}

real_inputs&
real_inputs::operator=(real_inputs& sens)
{
	this->no_inputs = sens.no_inputs;
	inputs.clear();
	for (int i=0; i<no_inputs; i++)
		this->inputs.push_back(sens.inputs[i]);
	return (*this);
}

real_inputs&
real_inputs::operator=(const real_inputs& sens)
{
	this->no_inputs = sens.no_inputs;
	inputs.clear();
	for (int i=0; i<no_inputs; i++)
		this->inputs.push_back(sens.inputs[i]);
	return (*this);
}

bool
real_inputs::operator==(const real_inputs& sens)
const
{
        assert(no_inputs == sens.no_inputs);
        for (int i=0; i<no_inputs; i++)
        {
                if (inputs[i]!=sens.inputs[i])
                        return false;
        }
        return true;
}

bool
real_inputs::operator!=(const real_inputs& sens)
const
{
        assert(no_inputs == sens.no_inputs);
        for (int i=0; i<no_inputs; i++)
        {
                if (inputs[i]!=sens.inputs[i])
                        return true;
        }
        return false;
}

void
real_inputs::numeric_representation(vector<double>& value)
const
{
        for (int i = 0; i<no_inputs; i++)
                value.push_back(inputs[i]);
}

void
real_inputs::set_numeric_representation(const vector<double>& value)
{
        inputs.clear();
        no_inputs = value.size();
        for (int i=0; i<no_inputs; i++)
                inputs.push_back(value[i]);
}

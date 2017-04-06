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
// Filename      : nlms.hpp
//
// Purpose       : Normalized Least Mean Square or Modified Delta Rule
//                 to compute prediction as in the original paper by 
//                 Stewart Wilson.
//
// Special Notes :
//
//
//
// Creator       : Pier Luca Lanzi and Daniele Loiacono
//
// Creation Date : 2008/11/24
//
// Current Owner : Pier Luca Lanzi
//-------------------------------------------------------------------------

#include <sstream>
#include <iostream>
#include <iterator>
#include "xcs_utility.hpp"
#include "xcs_random.hpp"
#include "pf/nlms.hpp"

using namespace std;

namespace xcsflib
{

bool	nlms_pf::init = false;
double	nlms_pf::learning_rate;
double	nlms_pf::xzero;


nlms_pf::nlms_pf( void * owner ) : base_pf( owner )
{
	assert(init);
	for (int i=0; i<=dimension; i++)
	{
		weights.push_back(0);

		//! random weight init
		// double w = xcs_random::random()*2-1;
		//weights.push_back(w);
	}
}

nlms_pf::nlms_pf(xcs_config_mgr2& xcs_config)
{
	if ( !nlms_pf::init )
	{
		if (!xcs_config.exist(tag_name()))
		{
			xcs_utility::error(class_name(), "constructor", "section <" + tag_name() + "> not found", 1);	
		}
		
		try {	string	str_random_weights;

			str_random_weights = (string) xcs_config.Value(tag_name(), "random weights", "off");
			xcs_utility::set_flag(str_random_weights,flag_random_weights);

			learning_rate = xcs_config.Value(tag_name(), "learning rate");

			xzero = xcs_config.Value(tag_name(), "x0");

#ifdef __DEBUG_LINEAR_WILSON__
			cout << "LR = " << learning_rate << endl;
			cout << "X0 = " << xzero << endl;
#endif

		} catch (const char *attribute) {
			string msg = "attribute \'" + string(attribute) + "\' not found in <" + tag_name() + ">";
			xcs_utility::error(class_name(), "constructor", msg, 1);
		}
	}	
	nlms_pf::init = true;
}

void nlms_pf::recombine (base_pf *f)
{
	/*! nothing done during recombination
	 *  we tried both mixing the weights and averaging them but the performance appers not to be influenced
	 */	
}

base_pf*
nlms_pf::clone(void *owner)
{
	nlms_pf *f = new nlms_pf(owner);

	f->weights.clear();

	for (int i=0; i<dimension+1; i++)
	{
		f->weights.push_back(this->weights[i]);
	}
	return f;
}

void 
nlms_pf::update(vector <double> &input, double target)
{
	double error = target - output(input);

	double xpow2 = xzero*xzero;
	for (int i=0;i<dimension;i++)
	{
		xpow2 += input[i]*input[i];
	}

	double correction = (learning_rate*error)/xpow2;

	weights[0] += xzero * correction;
	for (int i=0; i<dimension; i++)
	{
		weights[i+1]+=correction*input[i];
	}
}

void 
nlms_pf::print( ostream & output ) const
{
	output <<"[";
	for (int i=0; i<=dimension; i++)
	{
		if (i!=dimension)
			output << weights[i] << "\t";
		else
			output << weights[i] << "]";
	}
}

void 
nlms_pf::read( istream & input )
{
	weights.clear();
	weights.reserve(dimension);
	char dummy;

	input >> dummy >> weights[0];

	assert(dummy=='[');

	for (int i=1; i<=dimension; i++)
	{
		if (i!=dimension)
		{
			input >> weights[i];
		} else {
			input >> weights[i] >> dummy;
			assert(dummy==']');
		}
	}
}

//! output the prediction value
double 
nlms_pf::output (vector <double> &input)
{
	assert (input.size () == dimension);
	assert (input.size () == (weights.size()-1));

	double x = xzero * weights[0];
	for (int i = 0; i < dimension; i++)
	{
		x += input[i] * weights[i+1];
	}

	if (isnan(x))
	{
		assert(false);
		cout << "NAN - INPUT ";
 		copy (input.begin(), input.end(), ostream_iterator<double>(cout," "));
		cout << " - WEIGHTS ";
 		copy (weights.begin(), weights.end(), ostream_iterator<double>(cout," "));
		cout << endl;
	}
	return x;
};
	
//! latex string representing the prediction function
string 
nlms_pf::latex_equation(int precision) const
{
	ostringstream str;
	str.setf( ios::fixed );
	str.precision(precision);
	str << weights[0] * xzero;
	for(int i=1; i<=dimension; i++)
	{
		str << "+";
		str << weights[i];
		str << "\\times";
		str << "x_{" << i << "}";
	} 
	return str.str();
}

//! string representing the prediction function
string 
nlms_pf::equation() const
{
	ostringstream str;
	str << weights[0] << "*" << xzero;
	for(int i=1; i<=dimension; i++)
	{
		str << "+" << "x" << i;
		str << "*" << weights[i];
	} 
	return str.str();
}

//! put the weights to zero
void 
nlms_pf::clear ()
{
	for (int i = 0; i < dimension + 1; i++)
	{
		weights[i] = 0;
	}
}

} //! end xcsflib namespace

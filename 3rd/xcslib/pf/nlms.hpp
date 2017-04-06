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
#ifndef __PF_NLMS__
#define __PF_NLMS__

#include <cassert>
#include <iostream>

#include "base.hpp"
#include "xcs_config_mgr2.hpp"

namespace xcsflib
{

class nlms_pf : public base_pf
{

	private:

		//! true when the function has been initialized
		static bool	init;

		//! learning rate eta
		static double	learning_rate;

		//! x0
		static double	xzero;

		//! weight vector
		vector<double>	weights;

		//! true if weights are randomly initialized
		bool 		flag_random_weights;

	public:
		//! class name
		string class_name () const {return string ("xcsf::nlms_pf");};

		//! tag name
		string tag_name () const {return string ("prediction::nlms");};

		//! true if the class has been initialized
		bool inited() const {return init;};

		//! constructor 
		nlms_pf(xcs_config_mgr2 &xcs_config);

		//! constructor
		nlms_pf(void *owner);

		//! clone the current function
		base_pf *clone (void *owner = NULL);

		//! output the prediction value
		double output (vector < double >&input);
		
		//! parameter update based on input values and target value
		void update(vector <double> &input, double target);

		//! recombine function
		virtual void recombine (base_pf *f);
		
		//! mutate function (does nothing)
		base_pf *mutate () {assert(false);};

		//! print function to stream
		void print(ostream &output) const;

		//! read function from stream
		void read(istream &input);
		
		//! clear function parameters
		void clear();
		
		//! return a string representing the prediction function
		string equation() const;
		
		//! return a latex string for prediction function
		string latex_equation(int precision) const;
};

}	//! end namespace
#endif

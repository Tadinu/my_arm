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




#ifndef __BASE_AF__
#define __BASE_AF__

#include <cassert>
#include <cmath>
#include <iostream>
#include <sstream>

#include "xcs_config_mgr2.hpp"

namespace xcsflib
{
const unsigned int TYPE_NUM = 6;

typedef enum
{
	PREDICTION_NLMS,	//! normalized least mean square
	PREDICTION_RLS,		//! recursive least squares
	CONSTANT,	//! constant activation function
	ARCTG,
	SIGMOID,
	PREDICTION_LINEAR_WILSON,	//! linear approximator wh
	QUADRATIC_WH,			//! quadratic approximator wh
	CUBIC_WH,
	LINEAR_LS,
	QUADRATIC_LS,
	LINEAR_RLS,
	QUADRATIC_RLS,
	CUBIC_RLS,
	CUBIC_LS,
	LINEAR_NLMS,
	LINEAR_IDBD,
	LINEAR_K1,
	LINEAR_K2,
	LINEAR_IDD,
	CONST,
} t_function_type;

typedef enum
{
	NONE_MUTATION, RANDOM_MUTATION, DETERMINISTIC_MUTATION
} t_mutation_type;

typedef enum
{
	INIT_RANDOM, INIT_DEFAULT
}
t_init_mode;

class base_pf
{

private:
	//! true if the class has been inited
	static bool init;

protected:

	//! number of input variables 
	static unsigned long dimension;

	//! mutation type 
	static t_mutation_type mutation_type;

	//! initialization type 
	static t_init_mode init_mode;

	//! default prediction function
	static t_function_type default_function;

public:
	//! pointer to owner classifier
	void *owner;

	//! constructor
	base_pf ()
	{
		assert (init);	//! check if already inited
		owner = NULL;
	}

	//! destructor
	virtual ~base_pf(){};

	//! constructor based on the configuration file
	base_pf (xcs_config_mgr2& xcs_config);

	//! constructor
	base_pf (void *owner)
	{
		assert (init);		//! check if already inited
		this->owner = owner;	//! set owner
	};

	//! copy constructor
	base_pf (const base_pf & a)
	{
		assert (init);		//! check if already inited
		this->owner = a.owner;	//! set owner
	};

	//! class name
	virtual string class_name () const { return string ("xcsflib::base_pf"); };

	//! class tag
	virtual string tag_name () const { return string ("prediction::base"); };

	//! return an instance 
	friend base_pf *get_an_implementation (void *owner = NULL);

	//! return a copy of the current function
	virtual base_pf *clone (void *owner = NULL) { assert (false); };

	//! set the pointer to the owner classifier
	void set_owner (void *owner) { this->owner = owner; };

	//! return the pointer to the owner classifier
	void *get_owner () { return owner; };

	//! compute classifier prediction
	virtual double output (vector < double >&input) { assert (false); };

	//! update prediction function parameters according to a target and gradient
	virtual void update (vector<double> &input, double t, double g)	{ update(input, t); };
	virtual void update (vector<double> &input, double t)	{ assert (false); };

	//! pretty print the prediction function parameters to an output stream
	virtual void print (ostream & output) const { assert (false); };

	//! read the prediction function parameters from input stream
	virtual void read (istream & input) { assert (false); };

	//! mutate prediction function 
	virtual base_pf *mutate () { assert (false); };

	//! recombine prediction functions
	virtual void recombine (base_pf * f) { assert(false); };

	//! clear the parameters
	virtual void clear () { assert (false);	};

	//! return the equation corresponding to the prediction function
	virtual string equation () const { assert (false); };
	
	//! true if the class has been initialized
	virtual bool inited () const { return false; };

	//! number of inputs involved
	unsigned long dim() const {return dimension;};
};

t_mutation_type	get_mutation_type (string str);

t_function_type get_function_type (string str);

t_init_mode get_init_mode (string str);

bool *create_mask (string str);

t_function_type select_random_type (bool mask[]);

base_pf *type_to_function (t_function_type type, void *owner);

} //! end of xcsflib namespace

#endif

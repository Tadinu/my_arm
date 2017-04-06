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
// Filename      : action_base.hh
//
// Purpose       : base class to define actions
//                 
// Special Notes : 
//                 
//
// Creator       : Pier Luca Lanzi
//
// Creation Date : 2002/05/14
// Last Change   : 2003/06/23	checked 
// 		 : 2003/01/02	added print method
//               : 2002/05/27
//
// Current Owner : Pier Luca Lanzi
//-------------------------------------------------------------------------

/*! 
 * \file action_base.h
 *
 * \brief Base class to define actions. Fundamental methods are defined here.
 *
 * \author Pier Luca Lanzi
 *
 * \version 0.02
 *
 * \date 2003/06/23
 * 
 */

#ifndef __ACTION_BASE__
#define __ACTION_BASE__

#include <string>
#include "xcs_random.hpp"

/**
 * \class action_base
 *
 * \brief definition of the fundamental property of action class
 * 
 * Reinforcement learning assumes that there is a finite set of actions A.
 *
 */

template <class _action>
class action_base
{
protected:

	/*! \var unsigned long action
	 *  \brief index of the action represented
	 */
	unsigned long action;	

public:
	/*!
	 * \fn string class_name() const
	 * \brief name of the class that implements the action set
	 * 
	 * This function returns the name of the class. 
	 * Every class must implement this method that is used to 
	 * trace errors.
	 */

	virtual string class_name() const = 0;

	//! tag used to access the configuration file
	virtual string tag_name() const = 0;

	//! number of available actions (i.e., |A|)
	virtual unsigned long actions() const = 0;

	//! generate a random action 
	virtual void random() {set_value(xcs_random::dice(actions()));};

	//! mutate the action 
	virtual void mutate(const double& mu) = 0;

	//! set the action to the first available one (i.e., A0)
	virtual void reset_action() {set_value(0);}; 

	//! set the action to the next available one; it returns false if there are no more available actions.
	virtual bool next_action() 
	{ 
		//!
	 	if (value()+1<(actions())) 
		{
			set_value(value()+1);
			return true;
		} else {
			return false;
		}
	}; 

	//! return the integer action value
	virtual unsigned long value() const { return action; };

	//! set the action value
	virtual void set_value(unsigned long act) { action = act; };

	//! number of available actions
	virtual unsigned long	size() const { return actions(); };

	//! return the action value as a string
	virtual string string_value() const = 0;

	//! set the action value from a string
	virtual void set_string_value(string str) = 0;

	//! equality operator
	//bool operator==(const _action& act) const { return (value()==act.value()); };

	//@{ comparison operators
	
	//! equality operator
	virtual bool operator==(const _action& act) const { return (value()==act.value()); };
	//! less than operator
	virtual bool operator< (const _action& act) const { return (value()<act.value()); };
	//! not equal operator
	virtual bool operator!=(const _action& act) const { return (value()!=act.value()); };
	//@}

	//! read the action from an input stream 
	friend istream& operator>>(istream& input, _action& action) 
	{
		string	str;
		if(input >> str)
			action.set_string_value(str);
		return (input);
	};

	//! write the action to an output stream 
	friend ostream& operator<<(ostream& output, const _action& action)
	{
		string str;

		str = action.string_value();
		output << str;
		return (output);
	};

	//! assignment operators
	_action& operator=(_action& act) {set_value(act.value()); return (*this);};
	_action& operator=(const _action& act) {set_value(act.value()); return (*this);};

	//! print the action.
	virtual void print(ostream &output) const {output << string_value();};
	
	//! version
	string version() const {return "no information available";};
};
#endif

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
// Filename      : xcs_classifier.cc
//
// Purpose       : implementation of the methods for the class that implements XCS classifiers
//                 
// Special Notes : 
//                 
// Creator       : Pier Luca Lanzi
//
// Creation Date : 2002/06/03
//
// Current Owner : Pier Luca Lanzi
//-------------------------------------------------------------------------

#include "xcs_classifier.hpp"

/*! 
 *  \file xcs_classifier.cc
 *  \brief class implementation for XCS classifer
 *
 * \author Pier Luca Lanzi
 *
 * \version 0.01
 *
 * \date 2002/06/06
 * 
 */

unsigned long		xcs_classifier::id_count = 0;	
unsigned int		xcs_classifier::output_precision = 5;	


//! class constructor
xcs_classifier::xcs_classifier()
{
	set_initial_values();	
}

//! copy constructor
xcs_classifier::xcs_classifier(const xcs_classifier& classifier)
{
	set_initial_values();	
	condition = classifier.condition;
	action = classifier.action;
	prediction = classifier.prediction;		//!< prediction 
	error = classifier.error;			//!< prediction error
	fitness = classifier.fitness;			//!< classifier fitness
	actionset_size = classifier.actionset_size;	//!< estimate of the size of the action set [A]
	
	experience = classifier.experience;		//!< classifier experience, i.e., the number of times that the classifier has een updated
	numerosity = classifier.numerosity;		//!< classifier numerosity, i.e., the number of micro classifiers
	time_stamp = classifier.time_stamp;		//!< time of the last genetic algorithm application
}

//! copy constructor
xcs_classifier::xcs_classifier(xcs_classifier& classifier)
{
	set_initial_values();	
	condition = classifier.condition;
	action = classifier.action;
	prediction = classifier.prediction;		//!< prediction 
	error = classifier.error;			//!< prediction error
	fitness = classifier.fitness;			//!< classifier fitness
	actionset_size = classifier.actionset_size;	//!< estimate of the size of the action set [A]
	
	experience = classifier.experience;		//!< classifier experience, i.e., the number of times that the classifier has een updated
	numerosity = classifier.numerosity;		//!< classifier numerosity, i.e., the number of micro classifiers
	time_stamp = classifier.time_stamp;		//!< time of the last genetic algorithm application
}


bool 
operator==(const xcs_classifier& cl1, const xcs_classifier& cl2)
{
	return ((cl1.condition == cl2.condition) && (cl1.action == cl2.action));
}

bool 
operator!=(const xcs_classifier& cl1, const xcs_classifier& cl2)
{
	return !(cl1==cl2);
}

bool 
operator<(const xcs_classifier& cl1, const xcs_classifier& cl2)
{
	return ( (cl1.condition < cl2.condition) ||  
		 ((cl1.condition == cl2.condition) && (cl1.action < cl2.action)) );
}

ostream&
operator<<(ostream& os,const xcs_classifier& cs)
{
	os << cs.id() << '\t';
	os << cs.condition << " : " << cs.action << '\t';
	os.setf(ios::scientific);
	os.precision(xcs_classifier::output_precision);
	os << cs.prediction << '\t';
  	os.precision(xcs_classifier::output_precision);
  	os << cs.error << '\t';
  	os.precision(xcs_classifier::output_precision);
  	os << cs.fitness << '\t';
  	os.precision(xcs_classifier::output_precision);
  	os << cs.actionset_size << '\t';
  	os.precision(xcs_classifier::output_precision);
  	os << cs.experience << '\t';
  	os.precision(xcs_classifier::output_precision);
  	os << cs.numerosity << ' ';
	return (os);
}


//! read the classifier from an input stream
istream&
operator>>(istream& is, xcs_classifier& cs)
{
	char sep;
	if (!(is>>cs.identifier))
		return (is);

	if (!(is>>cs.condition))
		return (is);
		
	if (!(is>>sep))
		return (is);
		
	if (!(is>>cs.action))
		return (is);
		
	if (!(is>>cs.prediction))
		return (is);
		
	if (!(is >> cs.error))
		return (is);

	if (!(is >> cs.fitness))
		return (is);

	if (!(is >> cs.actionset_size))
		return (is);

	if (!(is >> cs.experience))
		return (is);

	is >> cs.numerosity;
	return (is);
}

//! pretty print the classifier to an output stream
void
xcs_classifier::print(ostream& output)
const
{
	output << identifier << "\t";
	condition.print(output);
	output << " : " << action << "\t";
	output.setf(ios::scientific);
	output.precision(xcs_classifier::output_precision);
	output << prediction << '\t';
  	output.precision(xcs_classifier::output_precision);
  	output << error << '\t';
  	output.precision(xcs_classifier::output_precision);
  	output << fitness << '\t';
  	output.precision(xcs_classifier::output_precision);
  	output << actionset_size << '\t';
  	output.precision(xcs_classifier::output_precision);
  	output << experience << '\t';
  	output.precision(xcs_classifier::output_precision);
  	output << numerosity << ' ';
}

bool	
xcs_classifier::match(const t_state& detectors)
{
	return (condition.match(detectors));
}

void	
xcs_classifier::random()
{
	condition.random();
	action.random();
	set_initial_values();
}

void	
xcs_classifier::cover(const t_state& detectors)
{
	condition.cover(detectors);
	action.random();
	set_initial_values();
}

void
xcs_classifier::mutate(const float mutationProb, const t_state& detectors) 
{
	condition.mutate(detectors,mutationProb);
	action.mutate(mutationProb);
}

void
xcs_classifier::recombine(xcs_classifier& classifier)
{
	condition.recombine(classifier.condition);
	swap(action,classifier.action);
}


bool 
xcs_classifier::subsume(const xcs_classifier& classifier)
const
{
	return ((action==classifier.action) && this->condition.is_more_general_than(classifier.condition));
};


inline
void 
xcs_classifier::set_initial_values()
{
	identifier = xcs_classifier::id_count++;
	numerosity = 1;
	time_stamp=0;
	experience=0;
	prediction=0;
	error=0;
	fitness=0;
	actionset_size=0;
}

//! assignment operator
xcs_classifier& 
xcs_classifier::operator=(xcs_classifier& classifier)
{
	set_initial_values();	
	condition = classifier.condition;
	action = classifier.action;
	prediction = classifier.prediction;		//!< prediction 
	error = classifier.error;			//!< prediction error
	fitness = classifier.fitness;			//!< classifier fitness
	actionset_size = classifier.actionset_size;	//!< estimate of the size of the action set [A]
	
	experience = classifier.experience;		//!< classifier experience, i.e., the number of times that the classifier has een updated
	numerosity = classifier.numerosity;		//!< classifier numerosity, i.e., the number of micro classifiers
	time_stamp = classifier.time_stamp;		//!< time of the last genetic algorithm application
	return *this;
}

//! assignment operator for a constant value
xcs_classifier& 
xcs_classifier::operator=(const xcs_classifier& classifier)
{
	set_initial_values();	
	condition = classifier.condition;
	action = classifier.action;
	prediction = classifier.prediction;		//!< prediction 
	error = classifier.error;			//!< prediction error
	fitness = classifier.fitness;			//!< classifier fitness
	actionset_size = classifier.actionset_size;	//!< estimate of the size of the action set [A]
	
	experience = classifier.experience;		//!< classifier experience, i.e., the number of times that the classifier has een updated
	numerosity = classifier.numerosity;		//!< classifier numerosity, i.e., the number of micro classifiers
	return *this;
}

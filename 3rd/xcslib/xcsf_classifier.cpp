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
// Filename      : xcsf_classifier.cc
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

#include "xcsf_classifier.hpp"

/*!
 * \file xcsf_classifier.cc
 * \brief class implementation for XCSF classifers
 *
 * \author Pier Luca Lanzi
 *
 * \version 0.01
 *
 * \date 2002/06/06
 *
 */

unsigned long xcsf_classifier::id_count = 0;
unsigned int xcsf_classifier::output_precision = 5;


//! class constructor
xcsf_classifier::xcsf_classifier()
{
	//! set classifier parameters
	set_initial_values();

	//! assign the activation function
	prediction_function = xcsflib::get_an_implementation(this);
}

//! copy constructor
xcsf_classifier::xcsf_classifier(const xcsf_classifier &classifier)
{
	set_initial_values();

	condition = classifier.condition;
	action = classifier.action;

	error = classifier.error; 			//!< prediction error
	fitness = classifier.fitness; 			//!< classifier fitness
	actionset_size = classifier.actionset_size; 	//!< action set size 
	
	experience = classifier.experience; 		//!< experience
	numerosity = classifier.numerosity; 		//!< numerosity
	time_stamp = classifier.time_stamp; 		//!< time of the last genetic algorithm application
	
	//! copy prediction function, 1st delete it then clone it from argument
	delete prediction_function;
	prediction_function = classifier.prediction_function->clone( this );
}

//! copy constructor
xcsf_classifier::xcsf_classifier( xcsf_classifier & classifier )
{
  set_initial_values();
  condition = classifier.condition;
  action = classifier.action;

  //#D v0.1
  //prediction = classifier.prediction;		//!< prediction

  error = classifier.error; //!< prediction error
  fitness = classifier.fitness; //!< classifier fitness
  actionset_size = classifier.actionset_size; //!< estimate of the size of the action set [A]

  experience = classifier.experience; //!< classifier experience, i.e., the number of times that the classifier has een updated
  numerosity = classifier.numerosity; //!< classifier numerosity, i.e., the number of micro classifiers
  time_stamp = classifier.time_stamp; //!< time of the last genetic algorithm application

  //#A BEGIN v0.1

  // clona l'prediction_function del classificatore
  //clear_af();
  prediction_function = classifier.prediction_function->clone( this );
  //#A END
}


bool 
operator==( const xcsf_classifier & cl1, const xcsf_classifier & cl2 )
{
	return ((cl1.condition==cl2.condition ) && (cl1.action==cl2.action));
}

bool operator != ( const xcsf_classifier & cl1, const xcsf_classifier & cl2 )
{
	return !( cl1 == cl2 );
}

bool 
operator < ( const xcsf_classifier & cl1, const xcsf_classifier & cl2 )
{
  return ( ( cl1.condition < cl2.condition ) || ( ( cl1.condition == cl2.condition ) && ( cl1.action < cl2.action ) ) );
}

ostream & operator << ( ostream & os, const xcsf_classifier & cs )
{
  os << cs.id() << '\t';
  os << cs.condition << " : " << cs.action << '\t';
  os.setf( ios::scientific );

  //#D BEGIN v0.1
  /*os.precision(xcsf_classifier::output_precision); os << cs.prediction << '\t'; */
  //#D END

  os.precision( xcsf_classifier::output_precision );
  os << cs.error << '\t';
  os.precision( xcsf_classifier::output_precision );
  os << cs.fitness << '\t';
  os.precision( xcsf_classifier::output_precision );
  os << cs.actionset_size << '\t';
  os.precision( xcsf_classifier::output_precision );
  os << cs.experience << '\t';
  os.precision( xcsf_classifier::output_precision );
  os << cs.numerosity << '\t';

  //#A BEGIN v0.1
  cs.prediction_function->print(os);
  //#A END

  return ( os );
}


//! read the classifier from an input stream
istream & operator >> ( istream & is, xcsf_classifier & cs )
{
	char sep;

	//! read identifier
	if ( !( is >> cs.identifier ) )
		return ( is );

	//! read condition
	if ( !( is >> cs.condition ) )
		return ( is );

	//! read separator 	
	if ( !( is >> sep ) )
		return ( is );

	//! read action 		
	if ( !( is >> cs.action ) )
		return ( is );

	//! read error
	if ( !( is >> cs.error ) )
		return ( is );

	//! read fitness
	if ( !( is >> cs.fitness ) )
		return ( is );

	//! read action set size
	if ( !( is >> cs.actionset_size ) )
		return ( is );
	
	//! read experience
	if ( !( is >> cs.experience ) )
		return ( is );

	//! read numerosity 	
	if (!(is >> cs.numerosity))
		return ( is );

	//! read activation function
	cs.prediction_function->read(is);

	return (is);
}

//! pretty print the classifier to an output stream
void xcsf_classifier::print( ostream & output ) const
{
	output << identifier << "\t";
	condition.print( output );

	output << " : " << action << "\t";

	//! set output format
	output.setf( ios::scientific );
	
	//! set precision and print parameters
	output.precision( xcsf_classifier::output_precision );
	output << error << '\t';

	output.precision( xcsf_classifier::output_precision );
	output << qerror << '\t';

	output.precision( xcsf_classifier::output_precision );
	output << fitness << '\t';
	output.precision( xcsf_classifier::output_precision );
	output << actionset_size << '\t';
	output.precision( xcsf_classifier::output_precision );
	output << experience << '\t';
	output.precision( xcsf_classifier::output_precision );
	output << numerosity << '\t';
	
	//! print the prediction function
	prediction_function->print(output);

	
}

bool xcsf_classifier::match( const t_state & detectors )
{
  return ( condition.match( detectors ) );
}

void xcsf_classifier::random()
{
  condition.random();
  action.random();
  set_initial_values();
}

void xcsf_classifier::cover( const t_state & detectors )
{
  condition.cover( detectors );
  action.random();
  set_initial_values();
}

void xcsf_classifier::mutate( const float mutationProb, const t_state & detectors )
{
  condition.mutate( detectors, mutationProb );
  action.mutate( mutationProb );

  //#A v0.1
  //prediction_function = prediction_function->mutate();
}

void xcsf_classifier::recombine( xcsf_classifier & classifier )
{
  condition.recombine( classifier.condition );
  swap( action, classifier.action );
  this->prediction_function->recombine(classifier.prediction_function);
}

bool xcsf_classifier::subsume( const xcsf_classifier & classifier ) const
{
//	cout << "Subsume with one parameter testing... ";
	return ( ( action == classifier.action ) && (this->condition.is_more_general_than( classifier.condition )) );
};

//! init parameters
inline void xcsf_classifier::set_initial_values()
{
	identifier = xcsf_classifier::id_count++;
	numerosity = 1;
	time_stamp = 0;
	experience = 0;
	
	error = 0;
	qerror = 0;
	fitness = 0;
	actionset_size = 0;
}

//! assignment operator
xcsf_classifier & xcsf_classifier::operator = ( xcsf_classifier & classifier )
{
	set_initial_values();				//!< init parameters

	condition = classifier.condition;		//!< condition
	action = classifier.action;			//!< action
	
	error = classifier.error;			//!< prediction error
	fitness = classifier.fitness; 			//!< classifier fitness
	actionset_size = classifier.actionset_size; 	//!< action set size 
	
	experience = classifier.experience; 		//!< experience
	numerosity = classifier.numerosity; 		//!< numerosity

	//! copy prediction function, 1st delete it then clone it from argument
	delete prediction_function;
	prediction_function = classifier.prediction_function->clone( this );
	
	return *this;
}

//! assignment operator for a constant value
xcsf_classifier & xcsf_classifier::operator = ( const xcsf_classifier & classifier )
{
	set_initial_values();				//!< init parameters

	condition = classifier.condition;		//!< condition
	action = classifier.action;			//!< action
	
	error = classifier.error;			//!< prediction error
	fitness = classifier.fitness; 			//!< classifier fitness
	actionset_size = classifier.actionset_size; 	//!< action set size 
	
	experience = classifier.experience; 		//!< experience
	numerosity = classifier.numerosity; 		//!< numerosity

	//! copy prediction function, 1st delete it then clone it from argument
	delete prediction_function;
	prediction_function = classifier.prediction_function->clone( this );
	
	return *this;
}

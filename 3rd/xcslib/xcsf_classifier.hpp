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
// Filename      : xcsf_classifier.hh
//
// Purpose       : definition of the class that implements XCS classifiers
//
// Special Notes :
//
//
//
// Creator       : Pier Luca Lanzi
//
// Creation Date : 2002/05/28
//
// Current Owner : Pier Luca Lanzi
//-------------------------------------------------------------------------
// Updates
//                 2004 12 31 added computed classifier prediction
//                 2003 08 07 general.hh becomes xcs_definitions
//                 2003 08 06 removed the id setting from the classifier
//                            constructor. Added two methods to explain
//                            fields produced/read by stream operators.
//-------------------------------------------------------------------------

/*!
 * \file xcsf_classifier.hpp
 *
 * \brief implement XCSF classifiers
 *
 */

//! include definitions for sensors, conditions, actions, environment
#include "xcs_definitions.hpp"
#include "pf/base.hpp"
#include "pf/utility.hpp"

//! general type definitions for XCS
#ifndef __XCSF_CLASSIFIER__
#define __XCSF_CLASSIFIER__

/*!
 * \class xcsf_classifier
 *
 * \brief class definition for XCSF classifiers
 */

class xcsf_classifier
{
public:
	//! name of the class that implements XCS classifiers
	/*!
	 * \fn string class_name() 
	 * \brief name of the class that implements the environment
	 * 
	 * This function returns the name of the class. 
	 * Every class must implement this method that is used to 
	 * trace errors.
	 */
	string class_name() const
	{
		return string( "xcsf_classifier" );
	};

	//! class tag used in the configuration file; currently not used
	string tag_name() const
	{
		return string( "xcsf_classifier" );
	};

public:
	//! class constructor
	xcsf_classifier();
	
	//! class constructor
	xcsf_classifier( const xcsf_classifier& );
	
	//! class copy constructor
	xcsf_classifier( xcsf_classifier& );
	
	//! class destructor
	~xcsf_classifier()
	{
		delete prediction_function;
	};

	//! equality operator
	bool operator == ( const xcsf_classifier & classifier )
	{
		return ( ( condition == classifier.condition ) && ( action == classifier.action ) );
	};

	//! inequality operator
	bool operator != ( const xcsf_classifier & classifier )
	{
		return ( ( action != classifier.action ) || ( condition != classifier.condition ) );
	};

	//! less than operator
	bool operator < ( const xcsf_classifier & classifier )
	{
		return ( ( condition < classifier.condition )
			|| ( ( condition == classifier.condition ) && ( action < classifier.action ) ) );
	};

	//! equality operator used for sorting classifiers according to the identifier
	bool operator == ( const unsigned long & id )
	{
		return ( identifier == id );
	};

	//! less than operator used for sorting classifiers according to the identifier
	bool operator < ( const unsigned long & id )
	{
		return ( identifier < id );
	};

	friend bool operator == ( const xcsf_classifier &, const xcsf_classifier & );
	friend bool operator < ( const xcsf_classifier &, const xcsf_classifier & );
	friend bool operator != ( const xcsf_classifier &, const xcsf_classifier & );

	//! assignment operator
	xcsf_classifier& operator = ( xcsf_classifier& );
	
	//! assignment operator for a constant value
	xcsf_classifier& operator = ( const xcsf_classifier& );
	
	//! return the label associated with the element in position fld returned by the stream operator
	string	stream_field( unsigned long fld ) const { };
	
	//! write the classifier to an output stream
	friend ostream & operator << ( ostream &, const xcsf_classifier & );
	
	//! read the classifier from an input stream
	friend istream & operator >> ( istream &, xcsf_classifier & );
	
	//! return the label associated with the element in position fld returned by the stream operator
	string print_field( unsigned long fld ) { };
	
	//! pretty print the classifier on the output stream named "output"
	void print( ostream & output ) const;
	
	//! save the state of the classifier class to an output stream
	static void save_state( ostream & output ) { output << id_count << endl; };
	
	//! restore the state of the classifier class from an input stream
	static void restore_state( istream & input ) { input >> id_count; };
	
	//! generate a random classifier
	void random();
	
	//! cover the current input state
	void cover(const t_state&);

	//! return true if the classifier matches the input state
	bool match(const t_state&);
	
	//! mutate the classifier according to the mutation probability "mu"
	void mutate( const float mutationProb, const t_state & );
	
	//! apply crossover between this classifier and another one
	void recombine( xcsf_classifier & classifier );
	
	//! return true if this classifier subsumes the classifier "cs"
	bool subsume( const xcsf_classifier & cs ) const;
	
	//! return the classifier id
	unsigned long id() const { return identifier; };
	
	//! generate the unique identifier (used when inserting the classifier in the population)
	void generate_id() { identifier = ++xcsf_classifier::id_count; };

	//! return computed classifier prediction
	double get_prediction(vector <double> input ){return prediction_function->output(input);};
	
	//! update classifier prediction parameters
	void update_prediction(vector <double> input, double target, double grad_term){prediction_function->update(input,target,grad_term);};
	void update_prediction(vector <double> input, double target){prediction_function->update(input,target);};

private:
	//!  set the classifier parameters to default values
	inline void set_initial_values();

public:
	bool is_more_general_than( xcsf_classifier cl ) const
	{
		return condition.is_more_general_than( cl.condition );
	};

private:
  	static unsigned long id_count; 		//!< global counter used to generate classifier identifiers
  	static unsigned int output_precision;	//!< precision of the output (useless)

public:
	unsigned long identifier; 		//!< classifier identifier; it is unique for each classifier

	t_condition condition;			//!< classifier condition
	t_action action; 			//!< classifier action

	xcsflib::base_pf *prediction_function;	//!< prediction function

	double error;				//!< prediction error
	double qerror;				//!< prediction quadratic error
	double fitness;				//!< classifier fitness
	double actionset_size;			//!< estimate of the size of the action set [A]
	
	unsigned long experience;		//!< classifier experience, i.e., the number of updates
	unsigned long numerosity;		//!< classifier numerosity, i.e., the number of micro classifiers
	unsigned long time_stamp;		//!< time of the last genetic algorithm application
};
#endif

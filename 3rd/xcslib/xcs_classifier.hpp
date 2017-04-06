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
// Filename      : xcs_classifier.hh
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
//                 2003 08 07 general.hh becomes xcs_definitions
//                 2003 08 06 removed the id setting from the classifier
//                            constructor. Added two methods to explain
//                            fields produced/read by stream operators.
//-------------------------------------------------------------------------


/*!
 * \file xcs_classifier.h
 *
 * \brief implement XCS classifiers
 *
 */

//! include definitions for sensors, conditions, actions, environment
#include "xcs_definitions.hpp"

//! general type definitions for XCS
#ifndef __XCS_CLASSIFIER__
#define __XCS_CLASSIFIER__

/*!
 * \class xcs_classifier
 *
 * \brief class definition for XCS classifiers
 */
class xcs_classifier
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
	string class_name() const { return string("xcs_classifier"); };

	//! class tag used in the configuration file; currently not used 
	string tag_name() const { return string("classifier"); };

 public:
	//! class constructor
	xcs_classifier();

	//! class constructor
	xcs_classifier(const xcs_classifier&);

	//! class constructor
	xcs_classifier(xcs_classifier&);

	//! class destructor
	~xcs_classifier() {};

	//! equality operator
	bool operator==(const xcs_classifier& classifier)
	{
		return ((condition==classifier.condition) && (action==classifier.action));
	};

	//! inequality operator
	bool operator!=(const xcs_classifier& classifier)
	{
		return ((action!=classifier.action) || (condition!=classifier.condition));
	};

	//! less than operator
	bool operator<(const xcs_classifier& classifier)
	{
		return ((condition<classifier.condition) ||
		        ((condition==classifier.condition) && (action<classifier.action)));
	};

	//! equality operator used for sorting classifiers according to the identifier
	bool operator==(const unsigned long& id)
	{
		return (identifier==id);
	};

	//! less than operator used for sorting classifiers according to the identifier
	bool operator<(const unsigned long& id)
	{
		return (identifier<id);
	};

	friend bool operator==(const xcs_classifier&, const xcs_classifier&);
	friend bool operator<(const xcs_classifier&, const xcs_classifier&);
	friend bool operator!=(const xcs_classifier&, const xcs_classifier&);

	//! assignment operator
	xcs_classifier& operator=(xcs_classifier&); 

	//! assignment operator for a constant value
	xcs_classifier& operator=(const xcs_classifier&);

	//! return the label associated with the element in position fld returned by the stream operator
	string	stream_field(unsigned long fld) {};

	//! write the classifier to an output stream
	friend ostream& operator <<(ostream&, const xcs_classifier&);

	//! read the classifier from an input stream
	friend istream& operator >>(istream&, xcs_classifier&);

	//! return the label associated with the element in position fld returned by the stream operator
	string	print_field(unsigned long fld) {};

	//! pretty print the classifier on the output stream named "output"
	void print(ostream& output) const;

	//! save the state of the classifier class to an output stream
	static void	save_state(ostream& output) { output << id_count << endl;};

	//! restore the state of the classifier class from an input stream
	static void	restore_state(istream& input) {input >> id_count;};

	//! generate a random classifier
	void	random();			

	//!
	void		cover(const t_state& input);
	//!
	bool	match(const t_state& detectors);

	//! mutate the classifier according to the mutation probability "mu"
	void	mutate(const float mutationProb, const t_state&); 

	//! apply crossover between this classifier and another one
	void	recombine(xcs_classifier& classifier);
	
	//! return true if this classifier subsumes the classifier "cs"
	bool	subsume(const xcs_classifier& cs) const;

	//! return the classifier id
	unsigned long	id() const {return identifier;};		
	
	//! generate the unique identifier (used when inserting the classifier in the population)
	void generate_id() {identifier = ++xcs_classifier::id_count;};	// 

 private:
	//!  set the classifier parameters to default values
	inline void set_initial_values();	

 public:
	bool is_more_general_than(xcs_classifier cl) const { return condition.is_more_general_than(cl.condition); };

 private:
	static 	unsigned long	id_count;		//!< global counter used to generate classifier identifiers
	static unsigned int	output_precision;	//!< precision of the output (useless)

 public:
	unsigned long		identifier;		//!< classifier identifier; it is unique for each classifier

	t_condition		condition;		//!< classifier condition
	t_action		action;			//!< classifier action

	double			prediction;		// //!< prediction 
	double			error;			// //!< prediction error
	double			fitness;		// //!< classifier fitness
	double			actionset_size;		// //!< estimate of the size of the action set [A]
	
	unsigned long		experience;		// //!< classifier experience, i.e., the number of times that the classifier has een updated
	unsigned long		numerosity;		// //!< classifier numerosity, i.e., the number of micro classifiers
	unsigned long		time_stamp;		// //!< time of the last genetic algorithm application
};
#endif

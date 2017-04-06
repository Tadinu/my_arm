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
// Filename      : interval.hh
//
// Purpose       : implements the template for intervals of generic type
//
// Special Notes :
//
//
// Creator       : Pier Luca Lanzi
//
// Creation Date :
//
// Current Owner :
//-------------------------------------------------------------------------

#define __INTERVAL_TEXT_FORMAT__ "[%lf;%lf]"

/*!
 * \file interval.h
 *
 * \author Pier Luca Lanzi
 *
 * \version 1.0
 *
 * \date
 *
 * \brief generic intervals
 *
 */

/*!
 * \class xcslib::interval<T> interval.h
 *
 * \brief definition class for generic intervals
 */

#ifndef __GENERIC_INTERVAL__
#define __GENERIC_INTERVAL__

#include <string>
#include <sstream>

using namespace std;

namespace xcslib {
	
	template<typename T>
	class interval
	{
	private:
		T lower_bound;
		T upper_bound;
	public:
		//! constructors
		
		//! empty constructor
		interval () {};
	
		//!
		interval (T lower, T upper)
		{
			lower_bound = lower;
			upper_bound = upper;
	
			assert(lower_bound <= upper_bound);
		};
	
		//! copy constructor
		interval (const interval<T>& i)
		{
			lower_bound = i.lower_bound;
			upper_bound = i.upper_bound;
		};
	
		//! destructor
		~interval<T> (){};
	
		//! set lower bound
		void set_lower_bound (T lower) 
		{
			lower_bound = lower;
			if (lower_bound > upper_bound)
				upper_bound=lower_bound;
		};
	
		//! set upper bound
		void set_upper_bound (T upper)
		{
			upper_bound = upper;
			if (lower_bound > upper_bound)
				lower_bound = upper_bound;
		};
	
		//! set lower bound
		T get_lower_bound () const {return lower_bound;};
	
		//! set upper bound
		T get_upper_bound () const {return upper_bound;};
	
		//! true when the interval is contained in interval i
		bool operator< (const interval<T>& i) const
		{
			return ((lower_bound >= i.get_lower_bound())&&(upper_bound <= i.get_upper_bound()));
		};
	
		//! equality operator
		bool operator==(const interval<T>& i) const
		{
			return ((lower_bound == i.get_lower_bound())&&(upper_bound == i.get_upper_bound()));
		};
	
		//! inequality operator
		bool operator!=(const interval<T>& i) const
		{
			return ((lower_bound != i.get_lower_bound())||(upper_bound != i.get_upper_bound()));
		};
	
		//! assignment operator
		interval<T>& operator=(interval<T>& i)
		{
			lower_bound = i.get_lower_bound();
			upper_bound = i.get_upper_bound();
			return (*this);
		};
	
		//! assignment operator for a constant value
		interval<T>& operator=(const interval<T>& i)
		{
			lower_bound = i.get_lower_bound();
			upper_bound = i.get_upper_bound();
			return (*this);
		};
	
		string string_value()
		const
		{
	#ifndef __INTERVAL_TEXT_FORMAT__
			ostringstream str_int;
	
			str_int << "[";
			str_int << lower_bound;
			str_int << ";";
			str_int << upper_bound;
			str_int << "]";
	
			return str_int.str();
	#else
			char c[500];
			if (!sprintf(c,__INTERVAL_TEXT_FORMAT__,lower_bound,upper_bound))
			{
				xcs_utility::error("real_interval","to_string()","real_interval text format not correct",1);
			}
			return string(c);
	#endif
		}
	
		void set_string_value(string str)
		{
	#ifndef __INTERVAL_TEXT_FORMAT__
			char ls, rs;	//! left and right squared parentheses
			char sep;	//! separator
	
			istringstream str_int(str);
	
			str_int >> ls;
			str_int >> lower_bound;
			str_int >> sep;
			str_int >> upper_bound;
			str_int >> rs;
	
			assert(lower_bound <= upper_bound);
			assert(ls=='[' && rs==']' && sep==';');
	#else
			if(!sscanf(str.c_str(),__INTERVAL_TEXT_FORMAT__,&lower_bound,&upper_bound))
			{
				xcs_utility::error("real_interval","to_string()","real_interval text format not correct",1);
			}
			assert(lower_bound <= upper_bound);
	// 		return (*this);
	#endif
		}
	
		void
		swap(interval<T>& i)
		{
			T tmp_lower = i.get_lower_bound();
			T tmp_upper = i.get_upper_bound();
		
			i.set_lower_bound(lower_bound);
			i.set_upper_bound(upper_bound);
		
			lower_bound = tmp_lower;
			upper_bound = tmp_upper;
		}
	
		void
		swap_lower(interval<T>& i)
		{
			T tmp_lower = i.get_lower_bound();
			i.set_lower_bound(lower_bound);
			lower_bound = tmp_lower;
			if (lower_bound > upper_bound)
				upper_bound = lower_bound;
		}
	
		void
		swap_upper(interval<T>& i)
		{
			T tmp_upper = i.get_upper_bound();
			i.set_upper_bound(upper_bound);
			upper_bound = tmp_upper;
			if (lower_bound > upper_bound)
				lower_bound = upper_bound;
		}
	};

};	//! end namespace xcslib
#endif

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



#include <cstdlib>
#include <cassert>
#include <string>
#include <sstream>
#include <iostream>

#include "generic.hpp"

using namespace xcslib;

generic::generic(std::string const& value) {
  value_=value;
}

generic::generic(const char* c) {
  value_=c;
}

generic::generic(double d) {
  std::stringstream s;
  s<<d;
  value_=s.str();
}

generic::generic(unsigned long d) {
  std::stringstream s;
  s<<d;
  value_=s.str();
}

generic::generic(long d) {
  std::stringstream s;
  s<<d;
  value_=s.str();
}

generic::generic(generic const& other) {
  value_=other.value_;
}

generic& generic::operator=(generic const& other) {
  value_=other.value_;
  return *this;
}

generic& generic::operator=(double i) {
  std::stringstream s;
  s << i;
  value_ = s.str();
  return *this;
}

generic& generic::operator=(std::string const& s) 
{
	value_=s;
	return *this;
}

generic::operator std::string() const 
{
	return value_;
}

generic::operator double() const 
{
#ifdef __DEBUG__
	std::cout << ">>STRING <" << value_.c_str() << "> TO " << atof(value_.c_str()) << std::endl;
#endif
	return atof(value_.c_str());
}


generic::operator long() const 
{
#ifdef __DEBUG__
	std::cout << ">>STRING <" << value_.c_str() << "> TO " << atol(value_.c_str()) << std::endl;
#endif
	return atol(value_.c_str());
}

generic::operator unsigned long() const 
{
	std::stringstream s(value_);
	unsigned long v;
	s>>v;
#ifdef __DEBUG__
	std::cout << ">>STRING <" << value_.c_str() << "> TO " << v << std::endl;
#endif
	return v;
}

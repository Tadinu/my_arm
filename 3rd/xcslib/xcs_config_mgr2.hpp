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



#ifndef __XCS_CONFIG_MGR2__
#define __XCS_CONFIG_MGR2__

#include <string>
#include <map>

#include "generic.hpp"
#include "xcs_utility.hpp"

class xcs_config_mgr2 {

	//! values in the configuration file
	std::map<std::string, xcslib::generic> 	content_;
	std::vector< pair<string,xcslib::generic> >	content2_;
	
	//! index of all the tags in the configuration file
	std::vector<std::string>		tags;
	
	//! extension of the configuration file
	std::string				file_extension;

public:	

	//! name of the class
	std::string class_name() const { return std::string("xcs_config_mgr"); };

	//! Constructor for the configuration manager class 
	/*!
	 *  This is the only constructor.
	 */
	xcs_config_mgr2(std::string const& extension);

	//! return the file extension that is used by the configuration manager
	string extension() const {return file_extension;};

	//! true if a section labeled tag has been found in the configuration file 
	bool exist(const std::string tag) const {return find(tags.begin(), tags.end(), tag)!=tags.end(); };

	//! save the current configuration to an output stream 
	void save(ostream&) const;
	
	xcslib::generic const& Value(std::string const& section, std::string const& entry) const;
	xcslib::generic const& Value(std::string const& section, std::string const& entry, double value);
	xcslib::generic const& Value(std::string const& section, std::string const& entry, std::string const& value);

};
#endif

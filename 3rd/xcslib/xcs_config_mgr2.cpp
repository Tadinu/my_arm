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



#include <fstream>
#include <iostream>

#include "xcs_config_mgr2.hpp"

xcs_config_mgr2::xcs_config_mgr2(std::string const& extension) 
{
	//! save file extension;
	file_extension = extension;

	//! open the configuration file
	string configFile = "confsys." + extension;
	std::ifstream file(configFile.c_str());

	if (!file.good())
	{
		string msg = "configuration file <" + configFile + "> not opened";
		xcs_utility::error(class_name(),"scan", msg, 1);
	}

#ifdef __DEBUG__
	std::cout << "OPENED <" << configFile << endl;
#endif
	
	//! clear the vector of tags
	tags.clear();
	
  	std::string line;
	std::string input_line;
	
  	std::string name;
  	std::string value;
  	std::string current_section;
  	std::string section_end;
  	
	int posEqual;
  	while (std::getline(file,input_line)) 
	{
		//! clear the line from leading and trailing spaces
		line=xcs_utility::trim(input_line);
		
    		//! does not consider empty lines
    		if (!line.length()) continue;

		//! does not consider comments
		if (line[0] == '#') continue;
		if (line[0] == ';') continue;
	
		//! check for the start/end of a section
		if (input_line[0] == '<') 
		{

			//! check if is an end
			if (input_line[1]=='/')
			{
				//! end of a section
				section_end=xcs_utility::trim(line.substr(2,line.find('>')-2));

				//cout << "*** " << current_section << endl;
				//cout << "*** " << section_end << endl;

				if (section_end!=current_section)
				{
					string msg = "section <"+current_section+"> ends with </"+section_end+">";
					xcs_utility::error(class_name(),"constructor", msg, 1);
				} else {
					//! clear current section
					current_section = "";
				}

			} else {

				//! start of a new section
				if (current_section!="")
				{
					xcs_utility::error(class_name(),"constructor", "nested section in <"+current_section+">", 1);
				}

				//! begin of a section
				current_section=xcs_utility::trim(line.substr(1,line.find('>')-1));
				tags.push_back(current_section);

				//! check for '/' symbols in tags
				if (current_section.find('/')!=std::string::npos)
				{
					string msg = "symbol '/' not allowed in tag " + section_end;
					xcs_utility::error(class_name(),"constructor", msg, 1);
				}
			}

#ifdef __DEBUG__
			std::cout << "TAG <" << inSection << ">" << endl;
#endif
			continue;
		}

		posEqual=line.find('=');
		
		if (posEqual==std::string::npos)
		{
			string msg = "configuration entry <" + line + "> not a comment, not a tag, not an assignment";
			xcs_utility::error(class_name(),"constructor", msg, 1);
		}
			
		name  = xcs_utility::trim(line.substr(0,posEqual));
		value = xcs_utility::trim(line.substr(posEqual+1));
			
#ifdef __DEBUG__
		std::cout << "\tATTRIBUTE <" << name << ">" << endl;
		std::cout << "\tVALUE     <" << value << ">" << endl;
#endif
		content2_.push_back(pair<string,xcslib::generic>(current_section+'/'+name, xcslib::generic(value)));
		content_[current_section+'/'+name]=xcslib::generic(value);
	}


}

void
xcs_config_mgr2::save(ostream &output) const
{
	for(vector<string>::const_iterator tag = tags.begin(); tag!=tags.end(); tag++)
	{
		string current_tag = *tag;
		
		output << "<" << current_tag << ">" << endl;
		
		for(vector< pair<string,xcslib::generic> >::const_iterator ci=content2_.begin(); ci!=content2_.end(); ci++)
		{
			string s = (ci->first);				
			
			string t = s.substr(0,s.find('/'));	//! tag in list
			string p = s.substr(s.find('/')+1);		//! associated generic

			/*output << "*** " << s << endl;
			output << "*** " << t << endl;
			output << "*** " << p << endl;*/

			xcslib::generic v = ci->second;			//! associated value
			
			if (current_tag == t)
			{
				output << "\t" << p << " = " << v << endl;
			}
			
		}
		
		output << "</" << current_tag << ">" << endl;
	}
}

xcslib::generic const& xcs_config_mgr2::Value(std::string const& section, std::string const& entry) const
{
#ifdef __DEBUG__
	for(std::map<std::string,xcslib::generic>::const_iterator ci=content_.begin(); ci!=content_.end(); ci++)
	{
		cout << ci->first << endl;
	}
#endif

	std::map<std::string,xcslib::generic>::const_iterator ci = content_.find(section + '/' + entry);

	if (ci == content_.end()) throw string(entry).c_str();
	return ci->second;
}

xcslib::generic const& xcs_config_mgr2::Value(std::string const& section, std::string const& entry, double value)
{
	try {
		return Value(section, entry);
	} catch(const char*) {
		return content_.insert(std::make_pair(section+'/'+entry, xcslib::generic(value))).first->second;
	}
}

xcslib::generic const& xcs_config_mgr2::Value(std::string const& section, std::string const& entry, std::string const& value)
{
	try {
		return Value(section, entry);
	} catch(const char*) {
		return content_.insert(std::make_pair(section+'/'+entry, xcslib::generic(value))).first->second;
	}
}

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
// Filename      : mm2.cc
//
// Purpose       : take a set of files containing real numbers which 
// 		          represent the plot of a certain variable and produce
// 		          an output file containing the average of all the input plots
//                 
// Special Notes : 
//                 
//
// Creator       : Pier Luca Lanzi
//
// Creation Date : 2004/09/22
//
// Current Owner : Pier Luca Lanzi
//
//-------------------------------------------------------------------------

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdlib>
#include <cmath>

using namespace std;

int mm2(int argc, char* argv[])
{
   if (argc<4)
   {  
	   cout << endl;
	   cout << "USAGE: mm <columns> <output file> <input files>" << endl;
	   cout << endl;
       cout << endl;
	   return 0;
   }
   
   unsigned long no_columns = atoi(argv[1]);	//! number of columns
   char* str_fn_output = argv[2];				//! output file
   unsigned long first_file = 3;				//! index to the first file
   unsigned long no_files = argc-first_file;	//! number of files
   
   cerr << "NUMBER OF COLUMNS: " << no_columns << endl;
   cerr << "OUTPUT FILE:       " << str_fn_output << endl;
   cerr << "NUMBER OF FILES    " << no_files << endl;
   cerr << "FILES:" << endl;
   for(unsigned long cf=first_file; cf<argc; cf++)
   {
	   cerr << "\t" << cf-first_file << "\t" << argv[cf] << endl;
   }
   
   vector<double> abscissa;		//! abscissa
   vector<double> avg;			//! vettore utilizzato per contenere medie
   vector<double> sq;			//! sum of squares used for standard deviation
   
   double elem; 		// per lettura elemento
   int index;			// scandisce files da processare
   int count; 			// conta il numero di elementi nel file
   int countMax;		// conta il numero di elementi massimo dei files
   int countMin;		// conta il numero di elementi minimo dei files
   char fileName[256];	// nome del file da processare	
      
   //! inizializzazione
   countMax = -1;			//! maximum no elements
   countMin = -1;			//! minimum no elements
   
   //! \var cf current file processed
   for (int cf=first_file; cf<argc; cf++)
   {
			ifstream IN(argv[cf]);

			if (!IN)
			{
				cerr << "\ainput file " << fileName;
				cerr << " not opened" << endl;
				exit(1);	
			}
			
			count=0;
			if (no_columns==1)
			{
				while (IN>>elem)
				{
					if (count>countMax)
					{  
						//! new element
				   		avg.push_back(elem);
						sq.push_back(elem*elem);
				   		countMax++;
					} else {  
						//! update avg and sq
				   		avg[count]+=elem;
						sq[count]+=elem*elem;
					};
					count++;
				};
			} else {
				double x;
				while (IN>>x>>elem)
				{
					if (count>countMax)
					{  
						//! new element
						abscissa.push_back(x);
				   		avg.push_back(elem);
						sq.push_back(elem*elem);
				   		countMax++;
					} else {  
						//! update avg and sq
						if (abscissa[count]!=x)
						{
							cerr << "FILE: \t" << argv[cf] << endl;
							cerr << "LINE: \t" << count << endl;
							cerr << "ERROR:\t" << x << "does not appear in previous files" << endl;
							cerr << endl;
							exit(-1);
						}
				   		avg[count]+=elem;
						sq[count]+=elem*elem;
					};
					count++;
				};
				
			}
			
			//! count was set to the next free line
			//! now count is set to the actual number of lines
			count--;	
			
			//! update countMin
			if ((countMin<0) || (count<countMin))
			{
				countMin=count;
			};
		};	
		
		cerr << "LINES :     " << count << endl;
		cerr << "MAX LINES : " << countMin << endl;
		cerr << "MIN LINES : " << countMax << endl;
		
	ofstream OUT(str_fn_output);
    if (!OUT)
	{
		cerr << "FILE: \t" << str_fn_output << endl;
		//cerr << "LINE: \t" << __line__ << endl;
		cerr << "ERROR:\t" << "not opened" << endl;
		cerr << endl;
		exit(-1);
	};	
	
	cout << "NO FILES " << no_files << endl;
	
    for (unsigned long el=0; el<=countMin; el++)
	{
		if (no_columns==2)
		{
			OUT << abscissa[el] << "\t";
		}
			
		//! average
		float average = avg[el]/double(no_files);
		OUT << average;
			
		//! variance
		double var = max(double(0),double((sq[el]/double(no_files))-(average*average)));
		
		//! standard deviation
		double sd = sqrt(var);
			
		//! standard error
		//cout << "NO FILES " << no_files << "\t" << double(no_files) << sqrt(double(no_files)) << endl;
		//cout << "SD = " << sd << endl;
		double se = sd/sqrt(double(no_files));
			
		//! output the standard error
		OUT << "\t" << se;
			
		//! standard error
		OUT << endl;
	}
	OUT.close();
}

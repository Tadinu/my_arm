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
#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

int moving_average(int argc, char* argv[])
{
   if (argc!=2)
   {  
	cerr << "USAGE: moving_agerage <window>" << endl;
   }
   else 
   {
      int winLen;
      winLen=atoi(argv[1]);

      double vect[winLen];
      double elem;
	  int index, count;
      double accumulatore;

      // init
      accumulatore=0.0;
      index=0;
      count=0;

      // add elements
      while (cin>>elem)
      {
         if (count>=winLen)
         {  // window is completed
            accumulatore-=vect[index];
            vect[index++]=elem;
            index%=winLen;
            accumulatore+=elem;
            cout << (accumulatore)/double(count) << endl;
         }
         else
         {
            // filling up the window
            vect[index++]=elem;
            index%=winLen;
            accumulatore+=elem;
            count++;
// AGGIUNTA //////////////////////////////////////
	    if (count==winLen)
	    {
		cout << (accumulatore)/double(count) << endl; 
	    }
//////////////////////////////////////////////////
         }
      }
   }
}

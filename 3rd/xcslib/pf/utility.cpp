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




#include <sstream>
#include <iostream>
#include <algorithm>
#include <iterator>

#include "pf/base.hpp"
#include "pf/utility.hpp"
#include "xcs_utility.hpp"
#include "xcs_random.hpp"

using namespace std;

namespace xcsflib
{
// 	t_function_type select_random_type( bool mask[] )
// 	{
// 
// 	}

	base_pf * type_to_function( t_function_type type, void  * owner )
	{
// 		if ( type == CONSTANT )
// 			return new constant_af( owner );
// 		if ( type == CONST )
// 			return new const_af( owner );

#ifdef __PF_RLS__
		if ( type == PREDICTION_RLS )
			return new rls_pf(owner);
#endif

#ifdef __PF_NLMS__
		if ( type == PREDICTION_NLMS )
			return new nlms_pf(owner);
#endif

// #ifdef __PF_LINEAR_WILSON__
// 		if ( type == PREDICTION_LINEAR_WILSON )
// 			return new linear_wilson_pf( owner );
// #endif

// 		if ( type == QUADRATIC_WH )
// 			return new quadratic_wh_af( owner );
// 		if ( type == CUBIC_WH )
// 			return new cubic_wh_af( owner );
// 
// 		if ( type == SINE_WH )
// 			return new sine_wh_af( owner );
// 
// 		if ( type == SINEF_WH )
// 			return new sinef_wh_af( owner );
// 
// 		//! recursive least squares
// 		if ( type == LINEAR_RLS )
// 			return new linear_rls_af( owner );      
// 		if ( type == QUADRATIC_RLS )
// 			return new quadratic_rls_af( owner );
// 		if ( type == CUBIC_RLS )
// 			return new cubic_rls_af( owner );
// 
// 
// 		//! gain adaptation
// 		if ( type == LINEAR_IDBD )
// 			return new linear_idbd_af( owner );
// 		if ( type == LINEAR_K1 )
// 			return new linear_k1_af( owner );
// 		if ( type == LINEAR_K2 )
// 			return new linear_k2_af( owner );
// 		if ( type == LINEAR_IDD )
// 			return new linear_idd_af( owner );
// 
// 		//! non linear
// 		if ( type == SIGMOID )
// 			return new sigmoid_af( owner );

		xcs_utility::error("utility", "type_to_function()", "type not allowed", 1);
	
	}

	t_mutation_type get_mutation_type( string str )
	{
		if ( str == "RANDOM" )
			return RANDOM_MUTATION;
		if ( str == "NONE" )
			return NONE_MUTATION;
		if ( str == "DETERMINISTIC" )
			return DETERMINISTIC_MUTATION;
	
		ostringstream msg;
		msg << "\"" << str << "\" is not a correct mutation type";
		xcs_utility::error( "activation", "get_mutation_type()", msg.str(), 1 );
	}

	t_init_mode get_init_mode( string str )
	{
		if ( str == "RANDOM" )
			return INIT_RANDOM;
		else if ( str == "default" )
			return INIT_DEFAULT;
		else {	
			ostringstream msg;
			msg << "\"" << str << "\" is not a correct init mode";
			xcs_utility::error( "activation", "get_init_type()", msg.str(), 1 );
		}
	}

	//! config the available prediction functions
	void config(xcs_config_mgr2& xcs_config)
	{
		//! init base class and all the ones available from the configuration file
		base_pf dummy_base(xcs_config);

		//! init all the prediction functions
		init_prediction_functions(xcs_config);
	};

	base_pf *get_an_implementation( void  * owner )
	{
		if ( base_pf::init_mode == INIT_DEFAULT )
			return type_to_function( base_pf::default_function, owner );

		assert(false);
/*
		if ( base_pf::init_mode == INIT_RANDOM )
		{
			t_function_type type = select_random_type( base_pf::function_mask );
			return type_to_function( type, owner );
		}
 */
	}

	//! return the function type associated to a name
	t_function_type
	get_function_type( string str )
	{
#ifdef __PF_NLMS__
		if ( str == "nlms" )
			return PREDICTION_NLMS;
#endif

#ifdef __PF_RLS__
		if ( str == "rls" )
			return PREDICTION_RLS;
#endif

		if ( str == "CONSTANT" )
			return CONSTANT;

		if ( str == "CONST" )
			return CONST;

		if ( str == "SIGMOID" )
			return SIGMOID;

		if ( str == "ARCTG" )
			return ARCTG;


		if ( str == "LINEAR_LS" )
			return LINEAR_LS;
		if ( str == "LINEAR_RLS" )
			return LINEAR_RLS;      
		if ( str == "QUADRATIC_LS" )
			return QUADRATIC_LS;
		if ( str == "QUADRATIC_RLS" )
			return QUADRATIC_RLS;
		if ( str == "QUADRATIC_WH" )
			return QUADRATIC_WH;

		if ( str == "CUBIC_WH" )
			return CUBIC_WH;

// 		if ( str == "SINE_WH" )
// 			return SINE_WH;

// 		if ( str == "SINEF_WH" )
// 			return SINEF_WH;

		if ( str == "CUBIC_LS" )
			return CUBIC_LS;

		if ( str == "LINEAR_NLMS" )
		{
			cout << "NLMS SELECTED" << endl;
			return LINEAR_NLMS;
		}

		if ( str == "LINEAR_IDBD" )
		{
			cout << "IDBD SELECTED" << endl;
			return LINEAR_IDBD;
		}

		if ( str == "LINEAR_K1" )
		{
			cout << "K1 SELECTED" << endl;
			return LINEAR_K1;
		}

		if ( str == "LINEAR_K2" )
		{
			cout << "K2 SELECTED" << endl;
			return LINEAR_K2;
		}

		if ( str == "LINEAR_IDD" )
		{
			cout << "IDD SELECTED" << endl;
			return LINEAR_IDD;
		}

		if ( str == "QUADRATIC_RLS" )
		{
			cout << "QUADRATIC RLS SELECTED" << endl;
			return QUADRATIC_RLS;
		}

		if ( str == "CUBIC_RLS" )
		{
			cout << "CUBIC RLS SELECTED" << endl;
			return CUBIC_RLS;
		}

		ostringstream msg;
		msg << "\"" << str << "\" is not a correct function type";
		xcs_utility::error( "activation", "get_function_type()", msg.str(), 1 );
	}

	//! init one prediction function
	void init_prediction_functions(xcs_config_mgr2 &xcs_config)
	{
// 		constant_af	dummy_constant(xcs_config);
// 		if (dummy_constant.inited())
// 		{
// 			cout << "\t\tconstant function" << endl;
// 			available_functions.insert(CONSTANT);
// 		}

// 		const_af	dummy_const(xcs_config);
// 		if (dummy_const.inited())
// 		{
// 			cout << "\t\tconst function" << endl;
// 			available_functions.insert(CONST);
// 		}

#ifdef __PF_RLS__
		rls_pf	dummy_rls(xcs_config);
		if (dummy_rls.inited())
		{
			//cout << "\t\tlinear function" << endl;
			available_functions.insert(PREDICTION_RLS);
		}
#endif

#ifdef __PF_NLMS__
		nlms_pf	dummy_nlms(xcs_config);
		if (dummy_nlms.inited())
		{
			//cout << "\t\tlinear function" << endl;
			available_functions.insert(PREDICTION_NLMS);
		}
#endif

#ifdef __PF_LINEAR_WILSON__
		linear_wilson_pf	dummy_linear_wilson(xcs_config);
		if (dummy_linear_wilson.inited())
		{
			//cout << "\t\tlinear function" << endl;
			available_functions.insert(PREDICTION_LINEAR_WILSON);
		}
#endif

// 		linear_ls_af		dummy_ls(xcs_config);    
// 		if (dummy_ls.inited())
// 		{
// 			cout << "\t\tlinear least square function" << endl;
// 			available_functions.insert(LINEAR_LS);
// 		}

// 		linear_rls_af		dummy_rls(xcs_config);        
// 		if (dummy_rls.inited())
// 		{
// 			cout << "\t\tlinear recursive least square function" << endl;
// 			available_functions.insert(LINEAR_RLS);
// 		}
// 
// 		quadratic_wh_af		dummy_wh_quadratic(xcs_config);
// 		if (dummy_wh_quadratic.inited())
// 		{
// 			cout << "\t\tquadratic widrow-hoff function" << endl;
// 			available_functions.insert(QUADRATIC_WH);
// 		}
// 
// 		cubic_wh_af		dummy_wh_cubic(xcs_config);
// 		if (dummy_wh_cubic.inited())
// 		{
// 			cout << "\t\tcubic widrow-hoff function" << endl;
// 			available_functions.insert(CUBIC_WH);
// 		}

// 		quadratic_ls_af		dummy_ls_quadratic(xcs_config);    
// 		if (dummy_ls_quadratic.inited())
// 		{
// 			cout << "\t\tquadratic linear least square function" << endl;
// 			available_functions.insert(QUADRATIC_LS);
// 		}

/*		quadratic_rls_af	dummy_rls_quadratic(xcs_config);    
		if (dummy_rls_quadratic.inited())
		{
			cout << "\t\tquadratic recursive least square function" << endl;
			available_functions.insert(QUADRATIC_RLS);
		}*/
	
// 		cubic_ls_af		dummy_ls_cubic(xcs_config);    
// 		if (dummy_ls_cubic.inited())
// 		{
// 			cout << "\t\tcubic linear least square function" << endl;
// 			available_functions.insert(CUBIC_LS);
// 		}

// 		sigmoid_af		dummy_sigmoid(xcs_config);        
// 		if (dummy_sigmoid.inited())
// 		{
// 			cout << "\t\tsigmoid function" << endl;
// 			available_functions.insert(SIGMOID);
// 		}

// 		linear_nlms_af		dummy_nlms(xcs_config);        
// 		if (dummy_nlms.inited())
// 		{
// 			cout << "\t\t *** nlms function" << endl;
// 			available_functions.insert(LINEAR_NLMS);
// 		}

// 		linear_idbd_af		dummy_idbd(xcs_config);        
// 		if (dummy_idbd.inited())
// 		{
// 			cout << "\t\t *** idbd function" << endl;
// 			available_functions.insert(LINEAR_IDBD);
// 		}
// 
// 		linear_k1_af		dummy_k1(xcs_config);        
// 		if (dummy_k1.inited())
// 		{
// 			cout << "\t\t *** k1 function" << endl;
// 			available_functions.insert(LINEAR_K1);
// 		}
// 
// 		linear_k2_af		dummy_k2(xcs_config);        
// 		if (dummy_k1.inited())
// 		{
// 			cout << "\t\t *** k2 function" << endl;
// 			available_functions.insert(LINEAR_K2);
// 		}
// 
// 		linear_idd_af		dummy_idd(xcs_config);        
// 		if (dummy_idd.inited())
// 		{
// 			cout << "\t\t *** idd function" << endl;
// 			available_functions.insert(LINEAR_IDD);
// 		}
// 
// 		quadratic_rls_af	dummy_qrls(xcs_config);        
// 		if (dummy_qrls.inited())
// 		{
// 			cout << "\t\t *** qrls function" << endl;
// 			available_functions.insert(QUADRATIC_RLS);
// 		}
// 
// 		cubic_rls_af		dummy_crls(xcs_config);        
// 		if (dummy_crls.inited())
// 		{
// 			cout << "\t\t *** crls function" << endl;
// 			available_functions.insert(CUBIC_RLS);
// 		}
// 
// 		sine_wh_af		dummy_wh_sine(xcs_config);
// 		if (dummy_wh_sine.inited())
// 		{
// 			cout << "\t\tsine widrow-hoff function" << endl;
// 			available_functions.insert(SINE_WH);
// 		}
// 
// 		sinef_wh_af		dummy_wh_sinef(xcs_config);
// 		if (dummy_wh_sinef.inited())
// 		{
// 			cout << "\t\tsinef widrow-hoff function" << endl;
// 			available_functions.insert(SINEF_WH);
// 		}
// 
// 		cout << "FUNCTIONS " << available_functions.size() << endl;
// 		copy(available_functions.begin(), available_functions.end(),        ostream_iterator<double>(cout, "\t"));
// 		cout << endl;
		if (available_functions.size()==0)
		{
			xcs_utility::error( "activation", "get_function_type()", "no prediction functions defined", 1 );
		}
	};
}

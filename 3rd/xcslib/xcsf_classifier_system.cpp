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
// Filename      : xcsf_classifier_system.cpp
//
// Purpose       : definition of the class that implements XCSF 
//                 
// Special Notes : 
//                 
// Creator       : Pier Luca Lanzi
//
// Creation Date : 2007/09/17
//
//-------------------------------------------------------------------------
// Updates
//-------------------------------------------------------------------------

/*! \file	xcsf_classifier_system.cpp
 *  \brief	Implementation of XCSF 
 *
 */

#include <cmath>
#include <iterator>
#include <string>
#include <fstream>
#include <algorithm>
#include "xcsf_classifier_system.hpp"

using namespace std;

//! reset all the collected statistics
void
xcs_statistics::reset()
{
	average_prediction = 0;
	average_fitness = 0;
	average_error = 0;
	average_actionset_size = 0;
	average_experience = 0;
	average_numerosity = 0;
	average_time_stamp = 0;
	average_no_updates = 0;
	system_error = 0;

	no_macroclassifiers = 0;
	no_ga = 0;
	no_cover = 0;
	no_subsumption = 0;
}

//! class constructor; it invokes the reset method \sa reset
xcs_statistics::xcs_statistics()
{
	reset();
}

ostream&
operator<<(ostream& output, const xcs_statistics& stats)
{
	output << stats.average_prediction << "\t";
	output << stats.average_fitness << "\t";
	output << stats.average_error << "\t";
	output << stats.average_actionset_size << "\t";
	output << stats.average_experience << "\t";
	output << stats.average_numerosity << "\t";
	output << stats.average_time_stamp << "\t";
	output << stats.average_no_updates << "\t";
	output << stats.system_error << "\t";

	output << stats.no_macroclassifiers << "\t";
	output << stats.no_ga << "\t";
	output << stats.no_cover << "\t";
	output << stats.no_subsumption << "\t";
	return (output);
}

istream&
operator>>(istream& input, xcs_statistics& stats)
{
	input >> stats.average_prediction;
	input >> stats.average_fitness;
	input >> stats.average_error;
	input >> stats.average_actionset_size;
	input >> stats.average_experience;
	input >> stats.average_numerosity;
	input >> stats.average_time_stamp;
	input >> stats.average_no_updates;
	input >> stats.system_error;

	input >> stats.no_macroclassifiers;
	input >> stats.no_ga;
	input >> stats.no_cover;
	input >> stats.no_subsumption;
	return (input);
}

xcsf_classifier_system::xcsf_classifier_system(xcs_config_mgr2& xcs_config)
{
	delta_del = .1;
	use_exponential_fitness = false;			//! deprecated

	string		str_covering_setting;
	string		str_ga_sub;				//! string to read the setting for GA subsumption
	string		str_as_sub;				//! string to read the setting for AS subsumption
	string		str_pop_init;				//! string to read the population init strategy
	string		str_exploration;			//! string to set exploration strategy
	string		DeleteWithAccuracyString;		//! string to set deletion strategy
	double		covering_threshold;			//! generic threshold for setting the covering strategy
	string		str_covering;				//! char array to read the covering strategy
	string 		str_error_first;			//! char array to read the update order for prediction error
	string		str_use_mam;				//! string for reading MAM settings
	string		str_ga_ts;				//! string for reading GA tournament selection settings
	string		str_discovery_component;		//! string for reading discovery component flag
	string		str_use_gd;				//! string for reading gradient descent flag
	string		str_update_test;			//! string for reading the update setting

	//! look for the init section in the configuration file
	if (!xcs_config.exist(tag_name()))
	{
		xcs_utility::error(class_name(), "constructor", "section <" + tag_name() + "> not found", 1);	
	}

	try {
		max_population_size = xcs_config.Value(tag_name(), "population size");
		learning_rate = xcs_config.Value(tag_name(), "learning rate");
		error_rate = xcs_config.Value(tag_name(), "error rate", learning_rate);
		discount_factor = xcs_config.Value(tag_name(), "discount factor");
		str_covering_setting = (string) xcs_config.Value(tag_name(), "covering strategy");
		str_covering = str_covering_setting.substr(0, str_covering_setting.find(" "));
		covering_threshold = atol(xcs_utility::trim(str_covering_setting.substr(0, str_covering_setting.find(" "))).c_str());
		str_discovery_component = (string) xcs_config.Value(tag_name(), "discovery component");
		theta_ga = xcs_config.Value(tag_name(), "theta GA");
		prob_crossover = xcs_config.Value(tag_name(), "crossover probability");
		prob_mutation = xcs_config.Value(tag_name(), "mutation probability");
		epsilon_zero = xcs_config.Value(tag_name(), "epsilon zero");
		vi = xcs_config.Value(tag_name(), "vi");
		alpha = xcs_config.Value(tag_name(), "alpha");
		init_prediction = xcs_config.Value(tag_name(), "prediction init");
		init_error = xcs_config.Value(tag_name(), "error init");
		init_fitness = xcs_config.Value(tag_name(), "fitness init");
		init_set_size = xcs_config.Value(tag_name(), "set size init");
		str_pop_init = (string) xcs_config.Value(tag_name(), "population init");

		str_exploration = (string) xcs_config.Value(tag_name(), "exploration strategy");
		DeleteWithAccuracyString = (string) xcs_config.Value(tag_name(), "deletion strategy");
		theta_del = xcs_config.Value(tag_name(), "theta delete");
		theta_sub = xcs_config.Value(tag_name(), "theta GA sub");
		theta_as_sub = xcs_config.Value(tag_name(), "theta AS sub");
		str_ga_sub = (string) xcs_config.Value(tag_name(), "GA subsumption");
		str_as_sub = (string) xcs_config.Value(tag_name(), "AS subsumption");
		str_error_first = (string) xcs_config.Value(tag_name(), "update error first");
		str_use_mam = (string) xcs_config.Value(tag_name(), "use MAM");
		str_ga_ts = (string) xcs_config.Value(tag_name(), "GA tournament selection");
		tournament_size = xcs_config.Value(tag_name(), "tournament size");

		str_use_gd = (string) xcs_config.Value(tag_name(), "gradient descent", "off");
		str_update_test = (string) xcs_config.Value(tag_name(), "update during test", "off");

	} catch (const char *attribute) {
		string msg = "attribute \'" + string(attribute) + "\' not found in <" + tag_name() + ">";
		xcs_utility::error(class_name(), "constructor", msg, 1);
	}

	//!
	set_covering_strategy(string(str_covering),covering_threshold);
	set_exploration_strategy(str_exploration.c_str());
	set_deletion_strategy(DeleteWithAccuracyString.c_str());
	set_init_strategy(string(str_pop_init));

	//! reserve memory for [P], [M], [A], [A]-1
	match_set.reserve(max_population_size);
	action_set.reserve(max_population_size);
	previous_action_set.reserve(max_population_size);
	select.reserve(max_population_size);

	//! set subsuption methods
	xcs_utility::set_flag(string(str_ga_sub),flag_ga_subsumption);
	xcs_utility::set_flag(string(str_as_sub),flag_as_subsumption);
	xcs_utility::set_flag(string(str_update_test), flag_update_test);
	//! create the prediction array
	create_prediction_array();
	
	//! check subsumption settings
	t_condition	cond;

	if (flag_as_subsumption && !cond.allow_as_subsumption())
	{
		xcs_utility::error( class_name(), "constructor", "AS subsumption requested but condition class does not allow", 1);
	}

	if (flag_ga_subsumption && !cond.allow_ga_subsumption())
	{
		xcs_utility::error( class_name(), "constructor", "GA subsumption requested but condition class does not allow", 1);
	}

	flag_cover_average_init = false;
	flag_ga_average_init = false;
	xcs_utility::set_flag(string(str_error_first), flag_error_update_first);
	xcs_utility::set_flag(string(str_use_mam), flag_use_mam);
	
	//! print the system configuration
	//print_options(clog);

	//! set tournament selection
	xcs_utility::set_flag(string(str_ga_ts), flag_ga_tournament_selection);
	xcs_utility::set_flag(string(str_discovery_component), flag_discovery_component);

	//! set gradient descent
	xcs_utility::set_flag(string(str_use_gd), flag_use_gradient_descent);

	flag_normalize_input = false;
	flag_use_gradient_descent = false;
}

void
xcsf_classifier_system::set_exploration_strategy(const char* explorationType)
{
	if (!strcmp(explorationType, "PROPORTIONAL"))
	{
		//!	action selection proportional to prediction array value
		action_selection_strategy = ACTSEL_PROPORTIONAL;
	} else if (!strcmp(explorationType, "UNIFORM"))
	{	//! 	random exploration
		action_selection_strategy = ACTSEL_SEMIUNIFORM;
		prob_random_action = 1.0;
	} else if (!strncmp(explorationType, "SEMIUNIFORM",11))
	{
		action_selection_strategy = ACTSEL_SEMIUNIFORM;
		prob_random_action = atof(explorationType+12);
		if ((prob_random_action<=0.) || (prob_random_action>1.))
		{
			//! probability out of range
			char errMsg[MSGSTR] = "";
			sprintf(errMsg,
			"'Biased' parameter (%f) out of range (0.0,1.0]",
				prob_random_action);
			xcs_utility::error(class_name(),"SetExploration",string(errMsg),1);
		}
	}
	else
	{	//! unknown exploration strategy
		char errMsg[MSGSTR] = "";
		sprintf(errMsg, "Unrecognized Exploration Policy '%s'", explorationType);
		xcs_utility::error(class_name(),"SetExploration",string(errMsg),1);
	}
}


void
xcsf_classifier_system::set_deletion_strategy(const char* deleteType)
{
	if (!strcmp(deleteType, "STANDARD"))
	{
		flag_delete_with_accuracy = false;

	}
	else if (!strcmp(deleteType, "ACCURACY-BASED"))
	{
		flag_delete_with_accuracy = true;
	}
	else
	{
		//! unrecognized deletion strategy
		char errMsg[MSGSTR] = "";
		sprintf(errMsg, "Unrecognized deletion strategy '%s'", deleteType);
		xcs_utility::error(class_name(),"set_deletion_strategy", string(errMsg), 1);
	}
}

void
xcsf_classifier_system::print_options(ostream& output) const
{
	output << "\nXCS OPTIONS\n" << endl;

	if (flag_ga_subsumption)
	{
		output << "\tGA subsumption:\t\tyes ";
		output << "\ttheta_ga       \t\t";
		output << theta_sub << "\n";
	}
	else
	{
		output << "\tGA subsumption:\t\tno ";
	}
	output << endl;

	if (flag_as_subsumption)
	{
		output << "\tAS subsumption:\t\tyes ";
	} else {
		output << "\tAS subsumption:\t\tno ";
	}
	output << endl;

	output << "\tpopulation initialization:\t";
	switch (population_init)
	{
		case INIT_EMPTY:
			output << "\t[P] is initially empty\n";
			break;
		case INIT_RANDOM:
			output << "\t[P] is initially random\n";
			break;
	}

	output << "\texploration strategy\t";
	output << endl;
	switch (action_selection_strategy)
	{
		case ACTSEL_SEMIUNIFORM:
			output << " biased ";
			output << "with probability " << prob_random_action << endl;
			break;
		case ACTSEL_UNIFORM:
			output << " uniform\n";
			break;
		case ACTSEL_PROPORTIONAL:
			output << " proportional\n";
			break;
	}

	output << "\tdeletion strategy\t";
	output << endl;
	output << (flag_delete_with_accuracy ? "\taccuracy-based\n" : "standard\n");
	output << endl;

	output << "\terror update:\t";
	output << (flag_error_update_first)?"error is updated first":"prediction is updated first";
	output << endl;
}

void
xcsf_classifier_system::init_classifier_set()
{
	t_classifier classifier;
	switch (population_init)
	{
		//! [P] = {}
		case INIT_EMPTY:
			erase_population();
			break;

		//! fill [P] with random classifiers
		case INIT_RANDOM:
			init_population_random();
			break;

		//! fill [P] with classifiers save in a file
		case INIT_LOAD:
			init_population_load(population_init_file);
			break;

		default:
			xcs_utility::error(class_name(),"init_classifier_set", "init strategy unknown" , 1);
	}
};


bool	compare_cl(t_classifier *clp1, t_classifier *clp2) {return *clp1<*clp2;};
void
xcsf_classifier_system::insert_classifier(t_classifier& new_cl)
{
	///CHECK
	assert(new_cl.actionset_size>=0);
	assert(new_cl.numerosity==1);

	//new_cl.condition.check_limits();
	///END CHECK

	/// keep a sorted index of classifiers
	t_classifier *clp = new t_classifier;
	*clp = new_cl;

	clp->time_stamp = total_steps;
	clp->experience = 0;

	t_set_iterator	pp;

	//! sequential search
	//pp = find(population.begin(),population.end(),clp);
	
	pp = lower_bound(population.begin(),population.end(),clp,compare_cl);
	if ((pp!=population.end()))
	{
		if ( (**pp)!=(*clp) )
		{
			clp->generate_id();
			population.insert(pp,clp);
			macro_size++;
		}
		else {
			(**pp).numerosity++;
			delete clp;
		}
	} else {
		population.insert(pp,clp);
		macro_size++;
	}
	population_size++;
}

void
xcsf_classifier_system::delete_classifier()
{
	t_set_iterator 	pp;

	double		average_fitness = 0.;
	double		vote_sum;
	double		vote;
	double		random;

	unsigned long	sel;

	//! check [P]
	//check("enter delete", clog);

	if (population_size<=max_population_size)
		return;

	for(pp=population.begin(); pp!=population.end(); pp++)
	{
		average_fitness += (**pp).fitness;
	}

	average_fitness /= ((double) population_size);

	select.clear();

	vote_sum = 0;
	for(pp=population.begin(); pp!=population.end(); pp++)
	{
		//! compute the deletion vote;
		vote = (**pp).actionset_size * (**pp).numerosity;

		if (flag_delete_with_accuracy)
		{
			if (((**pp).experience>theta_del) && ((((**pp).fitness)/double((**pp).numerosity))<delta_del*average_fitness))
			{
				vote = vote * average_fitness/(((**pp).fitness)/double((**pp).numerosity));
			}
		}
		vote_sum += vote;
		select.push_back(vote_sum);
	}

	random = vote_sum*(xcs_random::random());

	pp = population.begin();

	for(sel=0; ((sel<select.size()) && (random>select[sel])); sel++,pp++);

	assert(sel<select.size());

	if ((**pp).numerosity>1)
	{
		(**pp).numerosity--;
		population_size--;
	} else {

//#A BEGIN v0.2
#ifdef __DEBUG_CLASSIFIER__
                char debug_filename[256];
                sprintf (debug_filename,"%d.classifier",(**pp).identifier);
                ofstream debug;
                debug.open(debug_filename,ios::app);
                debug << endl << "DELETED" << endl;
                debug.close();
#endif
//#A END

		//	remove cl from [M], [A], and [A]-1
		t_set_iterator	clp;
		clp = find(action_set.begin(),action_set.end(), *pp);
		if (clp!=action_set.end())
		{
			action_set.erase(clp);
		}

		clp = find(match_set.begin(),match_set.end(), *pp);
		if (clp!=match_set.end())
		{
			match_set.erase(clp);
		}

		clp = find(previous_action_set.begin(),previous_action_set.end(), *pp);
		if (clp!=previous_action_set.end())
		{
			previous_action_set.erase(clp);
		}

		delete *pp;

		population.erase(pp);
		population_size--;
		macro_size--;
	}

	//CHECK
	//check("exit delete");
}

//! build [M]
unsigned long
xcsf_classifier_system::match(const t_state& detectors)
{
	t_set_iterator			pp;		/// iterator for visiting [P]
	unsigned long			sz = 0;		/// number of micro classifiers in [M]

	match_set.clear();				/// [M] = {}

	for(pp=population.begin();pp!=population.end();pp++)
	{
		if ((**pp).match(detectors))
		{
			match_set.push_back(*pp);
			sz += (**pp).numerosity;
       		}
   	}
	return sz;
}

//! perform covering on [M], only if needed
bool
xcsf_classifier_system::perform_covering(t_classifier_set &match_set, const t_state& detectors)
{
	switch (covering_strategy)
	{
		//! perform covering according to Wilson 1995
		case COVERING_STANDARD:
			return perform_standard_covering(match_set, detectors);
			break;

		//! covering strategy as in Butz and Wilson 2001
		case COVERING_ACTION_BASED:
			return perform_nma_covering(match_set, detectors);
			break;
		default:
			xcs_utility::error(class_name(),"perform_covering", "covering strategy not allowed", 1);
			exit(-1);
	}
}

//! perform covering based on the number of actions in [M]
bool
xcsf_classifier_system::perform_standard_covering(t_classifier_set &match_set, const t_state& detectors)
{
	if ((match_set.size()==0) || need_standard_covering(match_set, detectors))
	{
		t_classifier	classifier;

		//! create a covering classifier
		classifier.cover(detectors);

		//! init classifier parameters
		init_classifier(classifier);


		//! insert the new classifier in [P]
		insert_classifier(classifier);

		//! delete another classifier from [P] if necessary
		delete_classifier();

		//! signal that a covering operation took place
		return true;
	}
	return false;
}

bool
xcsf_classifier_system::need_standard_covering(t_classifier_set &match_set, const t_state& detectors)
{
	//unsigned long	i;
	t_set_iterator	pp;					//! iterator for visiting [P]
	//unsigned long	sz = 0;					//! number of micro classifiers in [M]
	double		average_prediction;			//!	average prediction in [P]
	double		total_match_set_prediction;		//!	total prediction in [M]

	if (match_set.size()==0)
		return true;

	average_prediction = 0;
	total_match_set_prediction = 0.;

	for(pp=population.begin();pp!=population.end();pp++)
	{
                //#A BEGIN v1.0
                vector<double> curr_input;

                // check if input must be normalied
                if (flag_normalize_input)
                {
                  vector<double> det_input;
                  detectors.numeric_representation(det_input);
                  (**pp).condition.normalize(det_input, curr_input);
                }
                else
                  detectors.numeric_representation(curr_input);
                //#A END

                //#M BEGIN v0.4
                average_prediction += (*pp)->get_prediction(curr_input) * (*pp)->numerosity;
                //#M END

		//cout << (*pp)->prediction << "*" << (*pp)->numerosity << "\t";
	}
	//cout << endl << endl;

	average_prediction = average_prediction/population_size;

	for(pp=match_set.begin(); pp!=match_set.end();pp++)
	{
                 //#A BEGIN v1.0
                 vector<double> curr_input;

                 // check if input must be normalied
                 if (flag_normalize_input)
                 {
                    vector<double> det_input;
                    detectors.numeric_representation(det_input);
                    (**pp).condition.normalize(det_input, curr_input);
                 }
                 else
                   detectors.numeric_representation(curr_input);
                 //#A END

                //#M BEGIN v0.4
                total_match_set_prediction += (*pp)->get_prediction(curr_input) * (*pp)->numerosity;
                //#M END

	}

	//cerr << "==> " << total_match_set_prediction << "<=" << fraction_for_covering << " x " << average_prediction << endl;;

	return (total_match_set_prediction<=fraction_for_covering*average_prediction);
}


//#M BEGIN v1.0
void
xcsf_classifier_system::build_prediction_array(vector<double>& current_input)
//#M END

{
	t_set_iterator					mp;
	vector<t_system_prediction>::iterator		pr;
	t_system_prediction				prediction;

	//! clear P(.)
	init_prediction_array();

	//! scan [M] and build the prediction array
	for(mp=match_set.begin(); mp!=match_set.end(); mp++ )
	{
		//!	look whether the action was already found in the prediction array
		pr = find(prediction_array.begin(), prediction_array.end(), ((**mp).action));

		if (pr==prediction_array.end())
		{
			xcs_utility::error(class_name(),"build_prediction_array", "action not found in prediction array", 1);
		} else {
			/*!	the action was already in the array
			 *	thus the corresponding value is updated
			 *	with the classifier values
			 */

                        //#A BEGIN v1.0
                        vector<double> curr_norm_input;

                        // check if input must be normalized
                        if (flag_normalize_input)
                            (**mp).condition.normalize(current_input, curr_norm_input);
                        else
                          curr_norm_input = current_input;
                        //#A END

                        //#M BEGIN v0.4
                        pr->payoff += (**mp).get_prediction(curr_norm_input) * (**mp).fitness;
                        //#M END

			pr->sum += (**mp).fitness;
			pr->n++;
		}
	}

	available_actions.clear();
	for(pr=prediction_array.begin(); pr!=prediction_array.end(); pr++ )
	{
		if (pr->n!=0)
		{
			available_actions.push_back((pr - prediction_array.begin()));
			if(pr->sum>0)
			{
				pr->payoff /= pr->sum;
			} else {
				if (pr->n>1)
				{
				//	print_set(match_set, cout);
				}
			}
		}
	}
};

void	xcsf_classifier_system::select_action(const t_action_selection policy, t_action& act)
{
	static unsigned long	stat_rnd = 0;
	//static unsigned long	stat_best = 0;
	//vector<t_system_prediction>::iterator	pr;
	vector<unsigned long>::iterator		best;
	vector<unsigned long>::iterator		ap;

	switch(policy)
	{
		//! select the action with the highest payoff
		case ACTSEL_DETERMINISTIC:
			assert(available_actions.size()>0);
			random_shuffle(available_actions.begin(),available_actions.end());

			best = available_actions.begin();
			for(ap=available_actions.begin(); ap!=available_actions.end(); ap++)
			{
				if ((prediction_array[*best].payoff) < (prediction_array[*ap].payoff))
					best = ap;
			}
			act = prediction_array[*best].action;
			break;

		//! biased action selection
		case ACTSEL_SEMIUNIFORM:
			assert(available_actions.size()>0);
			if (xcs_random::random()<prob_random_action)
			{	//	random action
				act = prediction_array[available_actions[xcs_random::dice(available_actions.size())]].action;
				stat_rnd++;
			} else {
				best = available_actions.begin();
				for(ap=available_actions.begin(); ap!=available_actions.end(); ap++)
				{
					if ((prediction_array[*best].payoff) < (prediction_array[*ap].payoff))
						best = ap;
				}
				act = prediction_array[*best].action;
			}
			break;
		default:
			xcs_utility::error(class_name(),"select_action", "action selection strategy not allowed", 1);
	};
};

//#M BEGIN v1.0
void
xcsf_classifier_system::update_set(const double P, t_classifier_set &action_set, vector<double>& current_input)
//#M END
{
	t_set_iterator	clp;
	double		set_size = 0;
        double          fit_sum=0;
        double          grad_term;

        //! update the experience of classifiers in [A]
	//! estimate the action set size
	for(clp=action_set.begin(); clp != action_set.end(); clp++)
	{
		(**clp).experience++;
		set_size += (**clp).numerosity;
                fit_sum += (**clp).fitness;
	}

	for(clp=action_set.begin(); clp!= action_set.end(); clp++)
	{
                //#A BEGIN v1.0
                vector<double> curr_norm_input;

                // check if input must be normalized
		if (flag_normalize_input)
			(**clp).condition.normalize(current_input, curr_norm_input);
		else
			curr_norm_input = current_input;

                double prediction = (**clp).get_prediction(curr_norm_input);

                //! prediction error is updated first if required (i.e., flag_error_update is true)
		if (flag_error_update_first)
		{
			double pe = fabs(P-prediction);

			//! update the classifier prediction error
			if (!flag_use_mam || ((**clp).experience>(1/learning_rate)))
			{
				(**clp).error += error_rate*(fabs(P- prediction)-(**clp).error);
				(**clp).qerror += error_rate*(pe*pe-(**clp).qerror);
			} else {
				(**clp).error += (fabs(P- prediction)-(**clp).error)/(**clp).experience;
				(**clp).qerror += (pe*pe-(**clp).qerror)/(**clp).experience;
			}
		}

                if (flag_gradient)
                    grad_term = (**clp).fitness / fit_sum;
                else
                    grad_term = 1;
                
                //! update classifier prediction: gradient is implemented only for nlms
                (**clp).update_prediction(curr_norm_input,P,grad_term);
		//(**clp).update_prediction(curr_norm_input,P);

                prediction = (**clp).get_prediction(curr_norm_input);

		if (!flag_error_update_first)
		{
			double pe = fabs(P-prediction);

                        //! update the classifier prediction error
			if (!flag_use_mam || ((**clp).experience>(1/learning_rate)))
			{
				(**clp).error += error_rate*(fabs(P- prediction)-(**clp).error);
				(**clp).qerror += error_rate*(pe*pe-(**clp).qerror);
			} else {
				(**clp).error += (fabs(P- prediction)-(**clp).error)/(**clp).experience;
				(**clp).qerror += (pe*pe-(**clp).qerror)/(**clp).experience;
			}
		}

		//! update the classifier action set size estimate
		if (!flag_use_mam || ((**clp).experience>(1/learning_rate)))
		{
			(**clp).actionset_size += learning_rate*(set_size - (**clp).actionset_size);
		} else {
			(**clp).actionset_size += (set_size - (**clp).actionset_size)/(**clp).experience;
		}

	}

	//! update fitness
	update_fitness(action_set);

	//! do AS subsumption
	if (flag_as_subsumption)
	{
		do_as_subsumption(action_set);
	}

}

void
xcsf_classifier_system::update_fitness(t_classifier_set &action_set)
{
	t_set_iterator 			as;
	double				ra;
	vector<double>			raw_accuracy;
	vector<double>::iterator	rp;
	double				accuracy_sum = 0;

	raw_accuracy.clear();

	for(as = action_set.begin(); as !=action_set.end(); as++)
	{
		if ((**as).error<epsilon_zero)
			ra = (**as).numerosity;
		else
			ra = alpha*(pow(((**as).error/epsilon_zero),-vi)) * (**as).numerosity;

		raw_accuracy.push_back(ra);
		accuracy_sum += ra;
	}

	//! to avoid underflow, if the sum of raw accuracy goes to zero, raw accuracy is used instead of relative accuracy

	if (!(accuracy_sum>0))
		accuracy_sum=1;

	for(as = action_set.begin(), rp=raw_accuracy.begin(); as!=action_set.end(); as++,rp++)
	{
		if (!flag_use_mam || ((**as).experience>(1/learning_rate)))
		{
			(**as).fitness += learning_rate*((*rp)/accuracy_sum - (**as).fitness);
		} else {
			(**as).fitness += ((*rp)/accuracy_sum - (**as).fitness)/(**as).experience;
		}
	}
}

bool
xcsf_classifier_system::subsume(const t_classifier &first, const t_classifier &second)
{
//	cout << "Entering subsume with two parameters..." << endl;

	bool	result;
	result = (classifier_could_subsume(first, epsilon_zero, theta_sub)) && (first.subsume(second));

//	cout << "The result of this subsume is " << result << endl;
//	cout << "True is " << true << endl;

	if (result)
		stats.no_subsumption++;

	return result;
}

bool
xcsf_classifier_system::need_ga(t_classifier_set &action_set, const bool flag_explore)
{
	double		average_set_stamp = 0;
	unsigned long	size = 0;

	if (!flag_explore) return false;

	t_set_iterator 	as;

	for(as=action_set.begin(); as!=action_set.end(); as++)
	{
		average_set_stamp += (**as).time_stamp * (**as).numerosity;
		size += (**as).numerosity;
	}

	average_set_stamp = average_set_stamp / size;

	return (fabs(total_steps - average_set_stamp)>=theta_ga);
}

//! check whether cl is subsumed by any of the classifiers in the action set
void
xcsf_classifier_system::ga_a_subsume(t_classifier_set &action_set, const t_classifier &cl, t_set_iterator &mg)
{
	t_set_iterator	as;
	
	mg = action_set.end();
	
	//! set the time stamp of classifiers in [A]
	for(as=action_set.begin(); (mg==action_set.end())&&(as!=action_set.end()); as++)
	{
		if (subsume(**as, cl))
		{
			mg = as;	
		}
	}
}

void
xcsf_classifier_system::genetic_algorithm(t_classifier_set &action_set, const t_state& detectors, const bool flag_condensation)
{
	t_set_iterator 	parent1;
	t_set_iterator	parent2;

	t_classifier	offspring1;
	t_classifier	offspring2;


	t_set_iterator	as;

	//! set the time stamp of classifiers in [A]
	for(as=action_set.begin(); as!=action_set.end(); as++)
	{
		(**as).time_stamp = total_steps;
	}

	//! select offspring classifiers
	if (!flag_ga_tournament_selection)
	{
		select_offspring(action_set, parent1, parent2);
	} else {
		select_offspring_ts(action_set, parent1);
		select_offspring_ts(action_set, parent2);
	}

	//! the GA is activated only if condensation is off
	if (!flag_condensation)
	{
		offspring1 = (**parent1);
		offspring2 = (**parent2);

		offspring1.numerosity = offspring2.numerosity = 1;
		offspring1.experience = offspring2.experience = 1;

		if (xcs_random::random()<prob_crossover)
		{
			offspring1.recombine(offspring2);
		}

		offspring1.mutate(prob_mutation,detectors);
		offspring2.mutate(prob_mutation,detectors);

		
		if (flag_ga_average_init)
		{
			//! init offspring classifiers with the averages of [A]
			init_classifier(offspring1,true);
			init_classifier(offspring2,true);

		} else {
			//! init offspring classifiers with the parents averages
			offspring1.error = offspring2.error = ((**parent1).error+(**parent2).error)/2;
			offspring1.fitness = offspring2.fitness = ((**parent1).fitness+(**parent2).fitness)/2;
			offspring1.time_stamp = offspring2.time_stamp = total_steps;
			offspring1.actionset_size = offspring2.actionset_size = ((**parent1).actionset_size + (**parent2).actionset_size)/2;
			offspring1.fitness = offspring1.fitness * 0.1;
			offspring2.fitness = offspring2.fitness * 0.1;
		}

		t_condition	cond;

//MOD
//		cout << "cond.allow_ga_subsumption= " << cond.allow_ga_subsumption() << endl;

		if (cond.allow_ga_subsumption() && flag_ga_subsumption)
		{
//MOD
//			cout << "Parent1 subsumes offsprimg1?= " << subsume(**parent1, offspring1) << endl;
//			cout << "Parent2 subsumes offsprimg1?= " << subsume(**parent2, offspring1) << endl;
//			cout << "Parent1 subsumes offsprimg2?= " << subsume(**parent1, offspring2) << endl;
//			cout << "Parent2 subsumes offsprimg2?= " << subsume(**parent2, offspring2) << endl;

			if (subsume(**parent1, offspring1))
			{	
				//! parent1 subsumes offspring1
				(**parent1).numerosity++;
				population_size++;
			} else if (subsume(**parent2, offspring1))
			{	
				//! parent2 subsumes offspring1
				(**parent2).numerosity++;
				population_size++;
			} else {
				//! offspring1 is not subsumbed by the parents
				if (!flag_gaa_subsumption)
				{
					//! if the usual GA subsumption is used, offspring classifier is inserted
					insert_classifier(offspring1);
				} else {
					//! if Martin's GA subsumption is used, offspring classifier is compared to the classifiers in [A]
					t_set_iterator 	par;

					ga_a_subsume(action_set,offspring1,par);
					if (par!=action_set.end())
					{				
						(**par).numerosity++;
						population_size++;
					} else {
						insert_classifier(offspring1);
					}
				}
			}
	
			if (subsume(**parent1, offspring2))
			{	// parent1 subsumes offspring2
				(**parent1).numerosity++;
				population_size++;
			}
			else if (subsume(**parent2, offspring2))
			{	// parent2 subsumes offspring2
				(**parent2).numerosity++;
				population_size++;
			} else {
				//! offspring1 is not subsumbed by the parents
				if (!flag_gaa_subsumption)
				{
					//! if the usual GA subsumption is used, offspring classifier is inserted
					insert_classifier(offspring1);
				} else {
					//! if Martin's GA subsumption is used, offspring classifier is compared to the classifiers in [A]
					t_set_iterator 	par;

					ga_a_subsume(action_set,offspring2,par);
					if (par!=action_set.end())
					{				
						(**par).numerosity++;
						population_size++;
					} else {
						insert_classifier(offspring2);
					}
				}
			}	
			delete_classifier();
			delete_classifier();
		} else {
			//! insert offspring classifiers without subsumption
			insert_classifier(offspring1);
			insert_classifier(offspring2);

			delete_classifier();
			delete_classifier();
		}

	} else {
		//! when in condensation
		(**parent1).numerosity++;
		population_size++;
		delete_classifier();

		(**parent2).numerosity++;
		population_size++;
		delete_classifier();
	}
}

void
xcsf_classifier_system::step(const bool exploration_mode, const bool condensationMode, const bool flag_update_test_problems)
{
	t_action	action;				//! selected action
	unsigned long	match_set_size;			//! number of microclassifiers in [M]
	//unsigned long	action_set_size;		//! number of microclassifiers in [A]
	double		P;				//! value for prediction update, computed as r + gamma * max P(.)
	//t_state	current_input;			//! current input from the environment
	double		max_prediction;

	//
	current_input = Environment->state();

	

	if (exploration_mode)
	{
		total_steps++;
	}

	/*!
	 * check if [M] needs covering,
	 * if it does, it apply the selected covering strategy, i.e., standard as defined in Wilson 1995,
	 * or action_based as defined in Butz and Wilson 2001
	 */

	//cout << "***" << endl;
	//print_set(population, cout);

#ifdef __DEBUG__
	cout << "STATE\t" << current_input << endl;
#endif
	do {
		match_set_size = match(current_input);
		//cout << "|[M]| = " << match_set_size << endl;
	}
   	while (perform_covering(match_set, current_input));

	
        //#M BEGIN v0.1
        //! build the prediction array P(.)
        vector<double> curr_input_vect;
        current_input.numeric_representation(curr_input_vect);
		build_prediction_array(curr_input_vect);
        //#M END


#ifdef __DEBUG__
	cout << "BUILT THE PREDICTION ARRAY" << endl;
	print_prediction_array(cout);
	cout << endl;
#endif

	//! select the action to be performed
	if (exploration_mode)
		select_action(action_selection_strategy, action);
	else
		select_action(ACTSEL_DETERMINISTIC, action);

	//! build [A]
	build_action_set(action);

#ifdef __DEBUG__
	cout << "ACTION " << action << endl;
#endif

	Environment->perform(action);

	//cout << "INPUT " << current_input << "->" << Environment->reward() << endl;
	//! if the environment is single step, the system error is collected
	if (Environment->single_step())
	{
			double payoff = prediction_array[action.value()].payoff;
			system_error = fabs(payoff-Environment->reward());

//			if (isnan(system_error))
//			{
//				//! 
//				print_set(action_set, cout, "NAN ");
//			}

			if (!(system_error>=0))
			{
				cout << "PAYOFF " << payoff << endl;
				cout << "REWARD " << Environment->reward() << endl;
				cout << "DIFF   " << payoff-Environment->reward() << endl;
				cout << "ABS    " << fabs(payoff-Environment->reward()) << endl;
			}
			//assert(system_error>=0);
	}

#ifdef __DEBUG__
	cout << "REWARD " << Environment->reward() << endl;
#endif

	total_reward = total_reward + Environment->reward();

        //! reinforcement component

	bool do_update;

	if (flag_update_test_problems)
	{
		do_update = true;
	} else {
		do_update = exploration_mode;
	}

	//! if [A]-1 is not empty it computes P
	if (previous_action_set.size())
	{
            
            
		vector<t_system_prediction>::iterator	pr = prediction_array.begin();
		max_prediction = pr->payoff;
                
                if (!flag_sarsa)
                {

		  for(pr = prediction_array.begin(); pr!=prediction_array.end(); pr++)
		  {
		        if (max_prediction<pr->payoff)
			{
				max_prediction = pr->payoff;
			}
		  }
                }
                
                else
                {
                    pr = find(prediction_array.begin(), prediction_array.end(), action);
                    if (pr==prediction_array.end())
                        xcs_utility::error(class_name(),"build_prediction_array", "action not found in prediction array", 1);
                        
                    max_prediction = pr->payoff;
                }
                
                
                    

		if (flag_bounded_payoff)
			max_prediction = min(max_prediction,double(0));
		P = previous_reward + discount_factor * max_prediction;

                //AVFSTAT << previous_input << "\t" << prev_action << "\t" << P << endl;

                //#M BEGIN v0.1
                //! use P to update the classifiers parameters
                if (do_update)
                {
                  vector<double> prev_input_vect;
                  previous_input.numeric_representation(prev_input_vect);
                  update_set(P, previous_action_set,prev_input_vect);
                }
                //#M END

	}

	if (Environment->stop())
	{
		P = Environment->reward();

                //#M BEGIN v0.1
                if (do_update)
                {
                  vector<double> curr_input_vect;
                  current_input.numeric_representation(curr_input_vect);
                  update_set(P, action_set,curr_input_vect);
                }
                //#M END

	}

	//! store the current input before performing the selected action
	/*!
	 * used by the genetic algorithm
	 */
	previous_input = current_input;

	//! apply the genetic algorithm to [A] if needed
	if (flag_discovery_component && need_ga(action_set, exploration_mode))
	{
		genetic_algorithm(action_set, previous_input, condensationMode);
		stats.no_ga++;
	}

	//!	[A]-1 <= [A]
	//!	r-1 <= r
	previous_action_set = action_set;
        prev_action = action;
	action_set.clear();
	previous_reward = Environment->reward();
}

void
xcsf_classifier_system::save_population(ostream& output)
{


	t_set_iterator		pp;
	t_classifier_set 	save;

	save = population;

	std::sort(save.begin(), save.end(), comp_numerosity());
	for(pp=save.begin(); pp!=save.end(); pp++)
	{
		(**pp).print(output);
		output << endl;
	}
}

void
xcsf_classifier_system::save_state(ostream& output)
{
	output << stats << endl;
	output << total_steps << endl;
	t_classifier::save_state(output);
	output << macro_size << endl;

	t_set_iterator	pp;

	for(pp=population.begin(); pp!=population.end(); pp++)
	{
		output << **pp << endl;
		output << endl;
	}
	output << endl;
}

void
xcsf_classifier_system::save_population_state(ostream& output)
{
	t_set_iterator	pp;

	for(pp=population.begin(); pp!=population.end(); pp++)
	{
		output << (**pp) << endl;
	}
}


void
xcsf_classifier_system::restore_state(istream& input)
{
	unsigned long size;
	input >> stats;
	input >> total_steps;
	t_classifier::restore_state(input);
	input >> size;

	population.clear();

    	t_classifier in_classifier;
	population_size = 0;
	macro_size = 0;

	for(unsigned long cl=0; cl<size; cl++)
	{
		if (!input.eof() && (input >> in_classifier))
		{
			t_classifier	*classifier = new t_classifier(in_classifier);
			population.push_back(classifier);
			population_size += classifier->numerosity;
			macro_size++;
		}
	};
	assert(macro_size==size);
}



//#A BEGIN v 1.1
void
xcsf_classifier_system::restore_population(istream& input)
{
        population.clear();

        t_classifier in_classifier;
        population_size = 0;
        macro_size = 0;

        while(!input.eof())
        {
                if (input >> in_classifier)
                {
                        t_classifier	*classifier = new t_classifier(in_classifier);
                        population.push_back(classifier);
                        population_size += classifier->numerosity;
                        macro_size++;
                }
        };
}
//#A END



//! defines what has to be done when a new experiment begins
void
xcsf_classifier_system::begin_experiment()
{
	//! reset the overall time step
	total_steps = 0;

	//! reset the number of overall learning steps
	total_learning_steps = 0;

	//! init the experiment statistics
	stats.reset();

	//! [P] contains 0 macro/micro classifiers
	population_size = 0;
	macro_size = 0;

	//! init [P]
	init_classifier_set();
}


//! defines what has to be done when a new problem begins.
void
xcsf_classifier_system::begin_problem()
{
	//! clear [A]-1
	previous_action_set.clear();

	//! clear [A]
	action_set.clear();

	//! set the steps within the problem to 0
	problem_steps = 0;

	//! clear the total reward gained
	total_reward = 0;

	//! added to clear the population
	/*while (population_size>max_population_size)
		delete_classifier();*/
}

//! defines what must be done when the current problem ends
void
xcsf_classifier_system::end_problem()
{

}



bool
xcsf_classifier_system::perform_nma_covering(t_classifier_set &match_set, const t_state& detectors)
{
	vector<t_system_prediction>::iterator	pr;
	t_action		act;
	unsigned long		total_actions = act.actions();
	unsigned long		covered_actions = total_actions;
	bool			covered_some_actions = false;		//! becomes true when covering classifiers are created

	//! clear the prediction array
	init_prediction_array();

        //#M BEGIN v
        //! build P(.)
        vector<double> curr_input_vect;
        current_input.numeric_representation(curr_input_vect);
	build_prediction_array(curr_input_vect);
        //#M END


	/*for(pr=prediction_array.begin(); pr!=prediction_array.end(); pr++ )
	{
		//!
		if (pr->n==0)
		{
			covered_actions--;
		}
	}
	*/

	//! the number of actions that are covered is computed as the number of available actions
	//! in the prediction array P(.)

	covered_actions = available_actions.size();

	covered_some_actions = false;

	if (covered_actions<tetha_nma)
	{
		for(pr=prediction_array.begin(); pr!=prediction_array.end() && (covered_actions<tetha_nma); pr++ )
		{
			//!
			if (pr->n==0)
			{
				t_classifier	classifier;

				classifier.cover(detectors);
				classifier.action = pr->action;

				init_classifier(classifier, flag_cover_average_init);

				insert_classifier(classifier);

				delete_classifier();

				covered_actions++;
			}
		}
		covered_some_actions = true;
	}

	return covered_some_actions;
};

void
xcsf_classifier_system::init_classifier(t_classifier& classifier, bool average)
{
	if (!average || (population_size==0))
	{

                //#D v0.1
                //classifier.prediction = init_prediction;

		classifier.error = init_error;
		classifier.fitness = init_fitness;

		classifier.actionset_size = init_set_size;
		classifier.experience = 0;
		classifier.time_stamp = total_steps;

		classifier.numerosity = 1;
	} else {
		t_set_iterator	cl;

		double		toterror = 0;
	   	double		totprediction = 0;
	   	double		totfitness = 0;
	   	double		totactionset_size = 0;
		unsigned long	popSize = 0;
		unsigned long	pop_sz = 0;

		for(cl=population.begin(); cl!=population.end(); cl++)
		{
			toterror += (**cl).error * (**cl).numerosity;

                        //#D v0.1
                        //totprediction += (**cl).prediction * (**cl).numerosity;

			totfitness += (**cl).fitness;
			totactionset_size += (**cl).actionset_size * (**cl).numerosity;
			popSize += (**cl).numerosity;
			pop_sz++;
		}


                //#D v0.1
                //classifier.prediction = totprediction/popSize;

		classifier.error = .25 * toterror/popSize;
		classifier.fitness = 0.1 * totfitness/pop_sz;
		classifier.actionset_size = totactionset_size/popSize;
		classifier.numerosity = 1;
		classifier.time_stamp = total_steps;
		assert(classifier.actionset_size>=0);
		assert(classifier.fitness>=0);
	};
}

//! build [A] from [M] and an action "act"
/*!
 * \param action selected action
 */
void
xcsf_classifier_system::build_action_set(const t_action& action)
{
	//! iterator in [M]
	t_set_iterator 	mp;

	//! clear [A]
	action_set.clear();

	//! fill [A] with the classifiers in [M] that have action "act"
	for( mp=match_set.begin(); mp!=match_set.end(); mp++ )
	{
		//if ((**mp).action==action)
		if ((*mp)->action==action)
		//if (true)
		{
			action_set.push_back(*mp);
		}
	}
	random_shuffle(action_set.begin(), action_set.end());
}

//! clear [P]
void
xcsf_classifier_system::clear_population()
{
	//! iterator in [P]
	t_set_iterator	pp;

	//! delete all the classifiers in [P]
	for(pp=population.begin(); pp!=population.end(); pp++)
	{
		delete *pp;
	}

	//! delete all the pointers in [P]
	population.clear();

	//! number of macro classifiers is set to 0
	macro_size = 0;

	//! number of micro classifiers is set to 0
	population_size = 0;
}

//! print a set of classifiers
/*!
 * \param set set of classifiers
 * \bug cause a memory leak!
 */
void
xcsf_classifier_system::print_set(t_classifier_set &set, ostream& output, string label)
{
	t_set_const_iterator	pp;
	t_classifier	cl;

	output << label << "================================================================================" << endl;
	for(pp=set.begin(); pp!=set.end(); pp++)
	{
		(**pp).print(output);
		output << label << endl;
	}
	output << label << "================================================================================" << endl;
}

//! check the integrity of the population
/*!
 * \param str comment that is printed before the trace information
 * \param output stream where the check information are printed
 */
void
xcsf_classifier_system::check(string str, ostream& output)
{
	//! iterator in [P]
	t_set_iterator	pp;

	//! check for the number of microclassifiers
	unsigned long check_population_size = 0;
	//! check for the number of macroclassifiers
	unsigned long check_macro_size = 0;

	//! count the number of micro and macro classifiers in [P]
	output << "CHECK <" << str << ">" << endl;
	output << "======================================================================" << endl;
	for(pp=population.begin(); pp!=population.end(); pp++)
	{
		check_population_size += (**pp).numerosity;
		check_macro_size ++;
	}
	output << "counter   = " << population_size << endl;
	output << "check     = " << check_population_size << endl;
	output << "limit     = " << max_population_size << endl;
	output  << "======================================================================" << endl;

	//! compare che check parameters to the current parameters
	assert(check_macro_size==macro_size);
	assert(check_population_size==population_size);
}


//! perform the operations needed at the end of the experiment
void
xcsf_classifier_system::end_experiment()
{

}

//@{
//! set the strategy to init [P] at the beginning of the experiment
/*!
 * \param strategy can be either \emph empty or \emph random
 * two strategies are allowed \emph empty set [P] to the empty set
 * \emph random fills [P] with random classifiers
 */
void
xcsf_classifier_system::set_init_strategy(string strategy)
{

#ifdef __DEBUG__
	cout << "STRATEGY " << strategy << endl;
	cout << "PREFIX " << strategy.substr(0,5)<< endl;
	cout << "SUFFIX " << strategy.substr(5)<< endl;
#endif

	if ( (strategy!="random") && (strategy!="empty") && (strategy.substr(0,5)!="load:") )
	{
		//!	unrecognized deletion strategy
		string	msg = "Unrecognized population init policy";
		xcs_utility::error(class_name(),"set_init_strategy", msg, 1);
	}

	if (strategy=="empty")
		population_init = INIT_EMPTY;
	else if (strategy=="random")
		population_init = INIT_RANDOM;
	else if (strategy.substr(0,5)=="load:")
	{
		population_init = INIT_LOAD;
		population_init_file = strategy.substr(5);
		cout << "LOAD FROM <" << population_init_file << ">" << endl;
	} else {
		//!	unrecognized deletion strategy
		string	msg = "Unrecognized population init policy";
		xcs_utility::error(class_name(),"set_init_strategy", msg, 1);
	}
}
//@}

//! select offspring classifiers from the population
//	20020722 modified the two for cycles so to continue from one to another
void
xcsf_classifier_system::select_offspring(t_classifier_set &action_set, t_set_iterator &clp1, t_set_iterator &clp2)
{
	t_set_iterator	as;		//! iterator in [A]
	vector<double>	select;		//! vector used to implement the roulette wheel
	unsigned long	sel;		//! counter

	double		fitness_sum;
	double		random1;
	double		random2;

	select.clear();

	fitness_sum = 0;
	for(as=action_set.begin(); as!=action_set.end(); as++)
	{
		fitness_sum += (**as).fitness;
		select.push_back( fitness_sum );
	}

	random1 = (xcs_random::random())*fitness_sum;
	random2 = (xcs_random::random())*fitness_sum;

	if (random1>random2)
		swap(random1,random2);

	for(sel = 0; (sel<select.size()) && (random1>=select[sel]); sel++);
	clp1 = action_set.begin()+sel;	// to be changed if list containers are used
	assert(sel<select.size());

	for(; (sel<select.size()) && (random2>=select[sel]); sel++);
	clp2 = action_set.begin()+sel;	// to be changed if list containers are used
	assert(sel<select.size());
}

void
xcsf_classifier_system::set_covering_strategy(const string strategy, double threshold)
{
	if (strategy=="standard")
	{
		covering_strategy = COVERING_STANDARD;
		fraction_for_covering = threshold;
	} else if (strategy=="action_based") {
		t_action	action;

		covering_strategy = COVERING_ACTION_BASED;

		//! the special value 0 for tetha_nma specifies that all the actions must be covered

		if (threshold==0)
			tetha_nma = action.actions();
		else
			tetha_nma = threshold;

		if (tetha_nma>action.actions())
		{
			xcs_utility::error(class_name(),"set_covering_strategy", "value for the threshold must not exceed the number of available actions", 1);

		}
	}
}

void
xcsf_classifier_system::create_prediction_array()
{
	t_action		action;
	t_system_prediction	prediction;

	//! clear the prediction array
	prediction_array.clear();

	//cout << "NUMBER OF ACTIONS " << action.actions() << endl;
	//!	build the prediction array with all the possible actions
	action.reset_action();
	do {
		//clog << "ADD ACTION IN PREDICTION ARRAY <" << action << ">" << endl;
		prediction.action = action;
		prediction.n = 0;
		prediction.payoff = 0;
		prediction.sum = 0;
		prediction_array.push_back(prediction);
	} while (action.next_action());

}

void
xcsf_classifier_system::init_prediction_array()
{
	vector<t_system_prediction>::iterator	sp;

	for(sp=prediction_array.begin(); sp!=prediction_array.end(); sp++)
	{
		sp->n =0;
		sp->payoff = 0;
		sp->sum = 0;
	}
}

void
xcsf_classifier_system::erase_population()
{
	t_set_iterator			pp;		//! iterator for visiting [P]
	for(pp=population.begin(); pp!=population.end(); pp++)
	{
		delete (*pp);
	}
	population.clear();

	population_size = 0;
}

//! delete a set of classifiers from [P], [M], [A], [A]-1
void
xcsf_classifier_system::as_subsume(t_set_iterator classifier, t_classifier_set &set)
{
	t_set_iterator	sp;		//! iterator for visiting the set of classifier
	t_set_iterator	pp;		//! iterator for visiting [P]

        //#A v0.2
        t_classifier *most_general;

//#A BEGIN v0.2
#ifdef __DEBUG_CLASSIFIER__
        unsigned long old_id = (**classifier).identifier;
        double old_error = (**classifier).error;
#endif
//#A END

        most_general = (*classifier);

	for(sp=set.begin(); sp!=set.end(); sp++)
	{
		//! remove cl from [M], [A], and [A]-1
		t_set_iterator	clp;
		clp = find(action_set.begin(),action_set.end(), *sp);
		if (clp!=action_set.end())
		{
			action_set.erase(clp);
		}

		clp = find(match_set.begin(),match_set.end(), *sp);
		if (clp!=match_set.end())
		{
			match_set.erase(clp);
		}

		clp = find(previous_action_set.begin(),previous_action_set.end(), *sp);
		if (clp!=previous_action_set.end())
		{
			previous_action_set.erase(clp);
		}


		pp = lower_bound(population.begin(),population.end(),*sp,compare_cl);
		if ((pp!=population.end()))
		{
			//! found another classifier, something is wrong
			if ( (*pp)!=(*sp) )
			{
				xcs_utility::error(class_name(),"as_subsumption", "classifier not found", 1);
				exit(-1);
			}
		}

		macro_size--;

                //#M BEGIN v0.2
                most_general->numerosity += (*pp)->numerosity;
                //#M END
//#A BEGIN v0.2
#ifdef __DEBUG_CLASSIFIER__
                char debug_filename[256];
                sprintf (debug_filename,"%d.classifier",(**pp).identifier);
                ofstream debug;
                debug.open(debug_filename,ios::app);
                debug << endl << "Subsumed by:" << endl;
                most_general->print(debug);
                debug << endl;
                debug.close();
                sprintf (debug_filename,"%d.subClassifier",most_general->identifier);
                debug.open(debug_filename,ios::app);
                (**pp).print(debug);
                debug << endl;
                debug.close();
#endif
//#A END

		delete *pp;
		population.erase(pp);
	}
	set.clear();
}

//! find the classifiers in set that are subsumed by the classifier
void
xcsf_classifier_system::find_as_subsumed(
	t_set_iterator classifier,
	t_classifier_set &set,
	t_classifier_set &subsumed)
const
{
	t_set_iterator	sp;	//! pointer to the elements in the classifier set

	subsumed.clear();

	if (classifier!=set.end())
	{
		for( sp=set.begin(); sp!=set.end(); sp++ )
		{
			if (*classifier!=*sp)
			{
				//if ( classifier_could_subsume( (**sp), epsilon_zero, theta_sub) &&
				//     (*classifier)->is_more_general_than(**sp))
				if ((*classifier)->is_more_general_than(**sp))
				{
					subsumed.push_back(*sp);
				}
			}
		}
	}
}

//! perform action set subsumption on the classifier in the set
void
xcsf_classifier_system::do_as_subsumption(t_classifier_set &set)
{
	/*!
	 * \brief check whether the condition type allow action set subsumption
	 *
	 * the same check is already performed at construction time.
	 * thus this might be deleted.
	 */
	t_condition	cond;
	if (!cond.allow_as_subsumption())
	{
		xcs_utility::error(class_name(),
			"do_as_subsumption",
			"condition does not allow action set subsumption", 1);
	}

	//! find the most general classifier
	t_set_iterator most_general;

	most_general = find_most_general(set);

	if ((most_general!=set.end()) && !classifier_could_subsume(**most_general, epsilon_zero, theta_sub))
	{
		xcs_utility::error(class_name(),
			"do_as_subsumption",
			"classifier could not subsume", 1);
	}


	//! if there is a "most general" classifier, it extracts all the subsumed classifiers
	if (most_general!=set.end())
	{
		t_classifier_set subsumed;

		find_as_subsumed(most_general, set, subsumed);

		if (subsumed.size())
		{
			as_subsume(most_general, subsumed);
		}
	}
}

xcsf_classifier_system::t_set_iterator
xcsf_classifier_system::find_most_general(t_classifier_set &set) const
{
	t_set_iterator 	most_general = set.end();

	t_set_iterator	sp;

	for( sp=set.begin(); sp!=set.end(); sp++ )
	{
		if ( classifier_could_subsume( (**sp), epsilon_zero, theta_as_sub) )
		{
			if (most_general==set.end())
				most_general = sp;
			else {
				if ( (*sp)->subsume(**most_general))

					most_general = sp;
			}
		}
	}

	return most_general;
}

void
xcsf_classifier_system::init_population_random()
{
	unsigned long	cl;
	timer		check;

	erase_population();

	check.start();
	for(cl=0; cl<max_population_size; cl++)
	{
		t_classifier *classifier = new t_classifier();
		classifier->random();
		init_classifier(*classifier);

		insert_classifier(*classifier);
	}
};

void
xcsf_classifier_system::print_prediction_array(ostream& output)
{
	vector<t_system_prediction>::iterator		pr;

	for(pr=prediction_array.begin(); pr!=prediction_array.end(); pr++)
	{
		output << pr->action << " " << pr->payoff << " ";
		//output << "(" << pr->action << ";" << pr->payoff << ")";
	}
}

void
xcsf_classifier_system::select_offspring_ts(t_classifier_set& set, t_set_iterator& clp)
{
	t_set_iterator	as;				//! iterator in set
	t_set_iterator	winner = set.end();

	while (winner==set.end())
	{
		for(as=set.begin(); as!=set.end(); as++)
		{
			bool selected = false;

			for(unsigned long num=0; (!selected && (num<(**as).numerosity)); num++)
			{
				if (xcs_random::random()<tournament_size)
				{
					if ((winner==set.end()) ||
					    (((**winner).fitness/(**winner).numerosity)<((**as).fitness/(**as).numerosity)))
					{
						winner = as;
						selected = true;
					}
				}
			}
		}
	}

	clp = winner;

}

void
xcsf_classifier_system::init_population_load(string filename)
{

	population.clear();		//! clear [P] before loading (20030808)
	
#ifdef GZSTREAM_H
	igzstream POPULATION(filename.c_str());
#else 
	assert(false);
	ifstream POPULATION(filename.c_str());
#endif

	if (!POPULATION.good())
	{
		xcs_utility::error( class_name(), "init_population_load", "file <"+filename+"> not found", 1);
	}
	
	t_classifier	in_classifier;
	unsigned long	n = 0;
	macro_size = 0;
	population_size = 0;

	while(!(POPULATION.eof()) && POPULATION>>in_classifier)
	{
// #ifdef __DEBUG__
// 		cout << "*** " << in_classifier << endl;
// #endif
		t_classifier	*classifier = new t_classifier(in_classifier);
		classifier->time_stamp = total_steps;
		population.push_back(classifier);
		population_size += classifier->numerosity;
		macro_size++;
	}
	sort(population.begin(),population.end());
	cout << "|[P]| = " << population.size() << endl;
	
	print_set(population,cout);
}


void
xcsf_classifier_system::trace(ostream& output)
{
  output << current_input << "\t";
  print_prediction_array(output);
}

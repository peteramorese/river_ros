#include<string>
#include<vector>
#include<iostream>
#include "edge.h"
#include "transitionSystem.h"

// Default constructor: One bag, one pickup location, one dropoff location. Propagation state
// is set by default. 
TransitionSystem::TransitionSystem() {
	state_map.clear();
	del_vec.clear();
	std::vector<std::string> location_labels_ = {"safep","safed","ee","L1_p","L2_p","L1_d","L2_d"};
	std::vector<std::string> obj_labels_ = {"Bag1"};
	domains.resize(2);
	domains[0].label = "pickup";
	domains[1].label = "dropoff";
	domains[0].locations.resize(2);
	domains[1].locations.resize(2);
	domains[0].locations = {"L1_p","L2_p"};
	domains[1].locations = {"L1_d","L2_d"};
	temp_state.setLocationLabels(location_labels_);
	temp_state.setObjLabels(obj_labels_);
	action_to_label_map['m'] = 0;
	action_to_label_map['g'] = 1;
	action_to_label_map['r'] = 2;
	action_to_label_map['t'] = 3;
	action_to_label_map_set = true;
	generate_river = true;
	init_state_ptr = new ManipulatorState;
	del_vec.push_back(init_state_ptr);
	init_state_ptr ->copyFrom(&temp_state);
	init_state_ptr ->setEELocation("safep");
	init_state_ptr ->setObjLocation("Bag1", "L1_p");
	//init_state_ptr ->setObjLocation("Bag2", "L2_p");
	//init_state_ptr = &init_state;
	init_state_set = true;
	//std::cout<<"heyoo:"<<std::endl;
}

// Constructor option 1: Locations "safep", "safed", and "ee" are by default added to the location
// label list, allowing the user to specify the object location labels. Propagation state
// is NOT set by default. NOTE: ALL OBJECT LOCATIONS THAT BELONG TO PICKUP DOMAIN MUST CONTAIN
// THE STRING "_p_" AND ALL OBJECT LOCATIONS THAT BELONG TO DROPOFF DOMAIN MUST CONTAIN
// THE STRING "_d_".

TransitionSystem::TransitionSystem(const std::vector<std::string>& obj_labels_, const std::vector<std::string>& obj_location_labels_) {
	state_map.clear();
	del_vec.clear();
	std::vector<std::string> location_labels_ = {"safep","safed","ee"};
	int N_obj_locations = obj_location_labels_.size();
	location_labels_.resize(N_obj_locations + 3);
	for (int i=3; i<location_labels_.size(); i++){
		location_labels_[i] = obj_location_labels_[i-3];
	}
	temp_state.setLocationLabels(location_labels_);
	temp_state.setObjLabels(obj_labels_);
	init_state_set = false;
	action_to_label_map['m'] = 0;
	action_to_label_map['g'] = 1;
	action_to_label_map['r'] = 2;
	action_to_label_map['t'] = 3;
	action_to_label_map_set = true;
	generate_river = true;
	ManipulatorState init_state;
	init_state.setEELocation("safep");
	init_state.setObjLocation("Bag1", "Lp");
}

// Constructor option 2: Nothing has been set by default. This will not work with 'generateRiver()'.
// A new transition system generation member must be created to use this constructor with settings
// that are not similar to those set in the other two constructor options.
/*
   TransitionSystem::TransitionSystem(const std::vector<std::string>& location_labels_, const std::vector<std::string>& obj_labels_) {
   state_map.clear();
   del_vec.clear();
   temp_state.setLocationLabels(location_labels_);
   temp_state.setObjLabels(obj_labels_);
   init_state_set = false;
   action_to_label_map_set = false;
   action_to_label_map['m'] = 0;
   action_to_label_map['g'] = 1;
   action_to_label_map['r'] = 2;
   action_to_label_map['t'] = 3;
   action_to_label_map_set = true;
   generate_river = false;
   }
   */


void TransitionSystem::setDomains(std::vector<domain> domains_){
	domains.clear();
	for (int i=0; i<domains_.size(); i++){
		domains.push_back(domains[i]);	
	}
}

std::string TransitionSystem::returnDomainLabel(std::string obj_location) const {
	std::string ret_string;
	ret_string = "NO_DOMAIN";
	for (int i=0; i<domains.size(); i++){
		bool in_domain = false;
		for (int j=0; j<domains[i].locations.size(); j++){
			if (domains[i].locations[j] == obj_location){
				in_domain = true;
				break;
			}
		}
		if (in_domain) {
			ret_string = domains[i].label;	
			break;
		}
	}
	return ret_string;
}

bool TransitionSystem::containsState(ManipulatorState* compare_state_ptr, unsigned int& contained_state_ind){
	bool ret_bool;
	contained_state_ind = 0; 
	for (int i=0; i<state_map.size(); i++){
		if (state_map[i]->isEqual(compare_state_ptr)){
			ret_bool = true;
			contained_state_ind = i;
			break;
		}
	}
}

void TransitionSystem::connectState(Edge* graph,unsigned int state_ind, float weight_, char action, ManipulatorState* state_ptr) {
	int label_ = action_to_label_map[action];
	unsigned int contained_state_ind;
	if (containsState(state_ptr, contained_state_ind)){
		graph->connect(state_ind, contained_state_ind, weight_, label_);
	} else {
		// If the state is not already contained, then push back the new state to the state
		// map, then connect current node to the new state (the last element in state_map)
		state_map.push_back(state_ptr);
		graph->connect(state_ind, state_map.size()-1 , weight_, label_);
	}
}

void TransitionSystem::resetPropagationNode(ManipulatorState* init_state_ptr_) {
	init_state_ptr = init_state_ptr_;	
	init_state_set = true;
}

void TransitionSystem::resetActionToLabelMap(const std::unordered_map<char, int>& action_to_label_map_) {
	action_to_label_map = action_to_label_map_;	
	action_to_label_map_set = true;
}

// A 'generateX' member can be added for any transition system being built around system-specific
// pre- and post-conditions. The pre- and post-conditions are coded within the main while loop. 
// This code is super disorganized and poorly written at the moment sorry.
void TransitionSystem::generateRiver(Edge* graph_ptr){
	if (init_state_set && action_to_label_map_set && generate_river){
		ManipulatorState* curr_state_ptr;
		ManipulatorState* add_state_ptr;

		// A queue is used to store states that still have to be analyzed. Initialized
		// with only the propagation state pointer (init_state_ptr)
		state_map.push_back(init_state_ptr);
		unsigned int state_ind = 0;
		while (state_ind < state_map.size()){
			curr_state_ptr = state_map[state_ind];
			//std::cout<<"boogie"<<std::endl;
			if (curr_state_ptr->isDefined()){
				//std::cout<<"i am defined"<<std::endl;
				//std::cout<<curr_state_ptr->returnEELocationLabel()<<std::endl; 
				if (curr_state_ptr->returnEELocationLabel() == "safep"){
					//std::cout<<"ee: safep"<<std::endl;
					for (int i=0; i<curr_state_ptr->returnNumLocations(); i++){
					//std::cout<<"yeet"<<std::endl;
						std::string temp_string = curr_state_ptr->returnLocationLabel(i);
						if (returnDomainLabel(temp_string) == "pickup"){
							if (curr_state_ptr->isGrabbing("ee")){
								if (!(curr_state_ptr->isOccupied(temp_string))){
									add_state_ptr = new ManipulatorState;	
									del_vec.push_back(add_state_ptr);
									add_state_ptr->copyFrom(curr_state_ptr);
									add_state_ptr->setEELocation(i);
									connectState(graph_ptr, state_ind, 1.0f, 'm', add_state_ptr);
								}
							} else {
								add_state_ptr = new ManipulatorState;	
								del_vec.push_back(add_state_ptr);
								add_state_ptr->copyFrom(curr_state_ptr);
								add_state_ptr->setEELocation(i);
								connectState(graph_ptr, state_ind, 1.0f, 'm', add_state_ptr);
							}
						}	
					}
					add_state_ptr = new ManipulatorState;	
					del_vec.push_back(add_state_ptr);
					add_state_ptr->copyFrom(curr_state_ptr);
					add_state_ptr->setEELocation("safed");
					connectState(graph_ptr, state_ind, 1.0f, 't', add_state_ptr);
				} else if (curr_state_ptr->returnEELocationLabel() == "safed"){
					//std::cout<<"ee: safed"<<std::endl;
					for (int i=0; i<curr_state_ptr->returnNumLocations(); i++){
						std::string temp_string = curr_state_ptr->returnLocationLabel(i);
						if (returnDomainLabel(temp_string) == "dropoff"){
							if (curr_state_ptr->isGrabbing("ee")){
								if (!(curr_state_ptr->isOccupied(temp_string))){
									add_state_ptr = new ManipulatorState;	
									del_vec.push_back(add_state_ptr);
									add_state_ptr->copyFrom(curr_state_ptr);
									add_state_ptr->setEELocation(i);
									connectState(graph_ptr, state_ind, 1.0f, 'm', add_state_ptr);
								}
							} else {
								add_state_ptr = new ManipulatorState;	
								del_vec.push_back(add_state_ptr);
								add_state_ptr->copyFrom(curr_state_ptr);
								add_state_ptr->setEELocation(i);
								connectState(graph_ptr, state_ind, 1.0f, 'm', add_state_ptr);
							}
						}	
					}
					add_state_ptr = new ManipulatorState;	
					del_vec.push_back(add_state_ptr);
					add_state_ptr->copyFrom(curr_state_ptr);
					add_state_ptr->setEELocation("safep");
					connectState(graph_ptr, state_ind, 1.0f, 't', add_state_ptr);
				} else {
					//std::cout<<"ee: obj location"<<std::endl;
					int grabbing_obj_ind;
					bool grabbing = curr_state_ptr->isGrabbing("ee", grabbing_obj_ind);
					if (grabbing){
						if (returnDomainLabel(curr_state_ptr->returnEELocationLabel()) == "pickup"){
							for (int i=0; i<curr_state_ptr->returnNumLocations(); i++){
								std::string temp_string = curr_state_ptr->returnLocationLabel(i);
								if ((returnDomainLabel(temp_string) == "pickup") && (temp_string != "ee")){
									if ((temp_string != curr_state_ptr->returnEELocationLabel())  && !(curr_state_ptr->isOccupied(i))) {
										add_state_ptr = new ManipulatorState;	
										del_vec.push_back(add_state_ptr);
										add_state_ptr->copyFrom(curr_state_ptr);
										add_state_ptr->setEELocation(i);
										connectState(graph_ptr, state_ind, 1.0f, 'm', add_state_ptr);
									}
								}
							}


						} else if (returnDomainLabel(curr_state_ptr->returnEELocationLabel()) == "dropoff"){
							for (int i=0; i<curr_state_ptr->returnNumLocations(); i++){
								std::string temp_string = curr_state_ptr->returnLocationLabel(i);
								if ((returnDomainLabel(temp_string) == "dropoff") && (temp_string != "ee")){
									if ((temp_string != curr_state_ptr->returnEELocationLabel())  && !(curr_state_ptr->isOccupied(i))) {
										add_state_ptr = new ManipulatorState;	
										del_vec.push_back(add_state_ptr);
										add_state_ptr->copyFrom(curr_state_ptr);
										add_state_ptr->setEELocation(i);
										connectState(graph_ptr, state_ind, 1.0f, 'm', add_state_ptr);
									}
								}
							}
						}
						add_state_ptr = new ManipulatorState;	
						del_vec.push_back(add_state_ptr);
						add_state_ptr->copyFrom(curr_state_ptr);
						add_state_ptr->setObjLocationToEELocation(grabbing_obj_ind);
						connectState(graph_ptr, state_ind, 1.0f, 'r', add_state_ptr);
					} else {
						if (returnDomainLabel(curr_state_ptr->returnEELocationLabel()) == "pickup"){
							for (int i=0; i<curr_state_ptr->returnNumLocations(); i++){
								std::string temp_string = curr_state_ptr->returnLocationLabel(i);
								if ((returnDomainLabel(temp_string) == "pickup") && (temp_string != "ee")){
									if ((temp_string != curr_state_ptr->returnEELocationLabel())) {
										add_state_ptr = new ManipulatorState;	
										del_vec.push_back(add_state_ptr);
										add_state_ptr->copyFrom(curr_state_ptr);
										add_state_ptr->setEELocation(i);
										connectState(graph_ptr, state_ind, 1.0f, 'm', add_state_ptr);
									}
								}
							}


						} else if (returnDomainLabel(curr_state_ptr->returnEELocationLabel()) == "dropoff"){
							for (int i=0; i<curr_state_ptr->returnNumLocations(); i++){
								std::string temp_string = curr_state_ptr->returnLocationLabel(i);
								if ((returnDomainLabel(temp_string) == "dropoff") && (temp_string != "ee")){
									if ((temp_string != curr_state_ptr->returnEELocationLabel())) {
										add_state_ptr = new ManipulatorState;	
										del_vec.push_back(add_state_ptr);
										add_state_ptr->copyFrom(curr_state_ptr);
										add_state_ptr->setEELocation(i);
										connectState(graph_ptr, state_ind, 1.0f, 'm', add_state_ptr);
									}
								}
							}
						}
						add_state_ptr = new ManipulatorState;	
						del_vec.push_back(add_state_ptr);
						add_state_ptr->copyFrom(curr_state_ptr);
						// Using index argument method for setObjLocation. 2
						// corresonds to "ee" in the line below
						add_state_ptr->setObjLocation(grabbing_obj_ind, 2);
						connectState(graph_ptr, state_ind, 1.0f, 'g', add_state_ptr);
					}
				}
			}
			std::cout<<"iteration: "<<state_ind<<std::endl;
			state_ind++;
		}
	} else {
		std::cout<<"Error: Must call 'setPropagationNode' before calling 'generate'\n";
	}
}

TransitionSystem::~TransitionSystem(){
	for (int i=0; i<del_vec.size(); i++){
		std::cout<<"deleting: "<<del_vec[i]<<std::endl;
		delete del_vec[i];
	}
}

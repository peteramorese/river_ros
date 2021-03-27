#pragma once
#include<string>
#include<vector>
#include<iostream>
#include<unordered_map>
#include "edge.h"
#include "state.h"

class TransitionSystem {
	private:
			
		// The indcies of state_map correspond to the node indices on the generated graph
		std::vector<ManipulatorState*> state_map;	
		unsigned int N_obj;
		ManipulatorState temp_state;
		ManipulatorState* init_state_ptr;
		bool init_state_set, action_to_label_map_set, generate_river;
		void addState(int ind_from, int ind_to, float weight_, char action, ManipulatorState* state_ptr);
		std::unordered_map<char, int> action_to_label_map;
		std::vector<ManipulatorState*> del_vec;
		struct domain {
			std::string label;
			std::vector<std::string> locations;
		};
		std::vector<domain> domains;
	public:
		TransitionSystem();
		TransitionSystem(const std::vector<std::string>& obj_labels_, const std::vector<std::string>& obj_location_labels_);
		/*
		TransitionSystem(const std::vector<std::string>& location_labels_, const std::vector<std::string>& obj_labels_);
		*/
		void setDomains(std::vector<domain> domains_);
		std::string returnDomainLabel(std::string obj_location) const;
		bool containsState(ManipulatorState* compare_state_ptr, unsigned int& contained_state_ind);
		void connectState(Edge* graph,unsigned int state_ind, float weight_, char action, ManipulatorState* state_ptr);
		void resetPropagationNode(ManipulatorState* init_state_ptr_);
		void resetActionToLabelMap(const std::unordered_map<char, int>& action_to_label_map_);
		void generateRiver(Edge* graph);
		~TransitionSystem();
};

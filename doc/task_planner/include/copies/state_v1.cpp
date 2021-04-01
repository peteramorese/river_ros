#include<string>
#include<vector>
#include<iostream>
#include<unordered_map>
#include "state.h"


/* State CLASS DEFINITION */

std::vector<std::string> State::location_labels;
const int State::UNDEF = -1;
bool State::labels_set;

int State::label2ind(std::string label, const std::vector<std::string>& labels) const {
	int ind = -1;
	for (int i=0; i<labels.size(); i++){
		if (labels[i] == label){
			ind = i;
			break;
		}
	}
	if (ind != -1){
		return ind;
	} else {
		std::cout<<"Error: label does not match any in labels set\n";
	}
}

void State::setLocationLabels(const std::vector<std::string>& location_labels_) {
	location_labels = location_labels_;
	labels_set = true;
}

int State::returnNumLocations() const {
	return location_labels.size();
}

std::string State::returnLocationLabel(int location_label_ind){
	return location_labels[location_label_ind];
}

bool State::isDefined() const {
	bool ret_bool = true;
	for (int i=0; i<state_space.size(); i++){
		if (state_space[i] == UNDEF){
			ret_bool = false;
			break;
		}		
	}
	return ret_bool;
}

void State::setState(std::vector<int> state_){
	if (labels_set){
		state_space = state_;
	} else {
		std::cout<<"Error: Set location labels before setting state\n";
	}
}

std::vector<int> State::getState() const {
	return state_space;
}

void State::copyTo(State* copy_state) const {
	copy_state->setState(state_space);
}

void State::copyFrom(const State* copy_state) {
	state_space = copy_state->getState();
}


/* ManipulatorState CLASS DEFINITION */
// Position 0 in the state space is reserved for the end effector, whereas all
// positions 1, 2, 3,... represent object 0, 1, 2,...

unsigned int ManipulatorState::N_obj;
std::vector<std::string> ManipulatorState::obj_labels;

std::string ManipulatorState::returnEELocationLabel() const {
	return location_labels[state_space[0]];
}

bool ManipulatorState::isGrabbing(std::string eef_flag) {
	bool ret_bool = false;
	for (int i=1; i<state_space.size(); i++){
		if (location_labels[state_space[i]] == eef_flag) {
			ret_bool = true;
			break;
		}
	}
	return ret_bool;
}

bool ManipulatorState::isGrabbing(std::string eef_flag, int& grabbing_obj_ind) {
	bool ret_bool = false;
	grabbing_obj_ind = -1;
	for (int i=1; i<state_space.size(); i++){
		if (location_labels[state_space[i]] == eef_flag) {
			grabbing_obj_ind = i - 1;
			ret_bool = true;
			break;
		}
	}
	return ret_bool;
}

int ManipulatorState::returnEELocation() const {
	return state_space[0];
}

void ManipulatorState::setEELocation(std::string location_label) {
	int ind = label2ind(location_label, location_labels);
	state_space[0] = ind;
}

void ManipulatorState::setEELocation(int location_label_ind) {
	state_space[0] = location_label_ind;
}

void ManipulatorState::setObjLocationToEELocation(int obj_label_ind) {
	state_space[obj_label_ind] = state_space[0];
}

void ManipulatorState::setObjLabels(const std::vector<std::string>& obj_labels_) {
	N_obj = obj_labels_.size();
	state_space.resize(N_obj+1);	
	obj_labels = obj_labels_;
}


std::string ManipulatorState::returnObjLocation(std::string obj_label) const {
	std::string ret_string;
	int obj_label_ind = label2ind(obj_label, obj_labels);
	if (state_space[obj_label_ind+1] != UNDEF){
		ret_string = location_labels[state_space[obj_label_ind+1]];
	} else {
		ret_string = UNDEF_label;	
	}
	return ret_string;
}

int ManipulatorState::returnObjLocation(int obj_label_ind) const {
	if (obj_label_ind < 0 || obj_label_ind > obj_labels.size()){
		std::cout<<"Error: Obj Label Index out of bounds\n";
	} else {
		if (state_space[obj_label_ind+1] == UNDEF){
			return UNDEF;
		} else {
			return state_space[obj_label_ind+1];
		}
	}
}

void ManipulatorState::setObjLocation(std::string obj_label, std::string location_label){
	int obj_label_ind = label2ind(obj_label, obj_labels);
	int location_label_ind = label2ind(location_label, location_labels);
	state_space[obj_label_ind+1] = location_label_ind;
}

void ManipulatorState::setObjLocation(int obj_label_ind, int location_label_ind){
	state_space[obj_label_ind+1] = location_label_ind;
}

bool ManipulatorState::isOccupied(std::string location_label) const {
	int location_label_ind = label2ind(location_label, location_labels);
	bool ret_bool = false;
	for (int i=1; i<state_space.size(); i++){
		if (state_space[i] == location_label_ind) {
			ret_bool = true;
			break;
		}
	}
	return ret_bool;
}

bool ManipulatorState::isOccupied(int location_label_ind) const {
	bool ret_bool = false;
	for (int i=1; i<state_space.size(); i++){
		if (state_space[i] == location_label_ind) {
			ret_bool = true;
			break;
		}
	}
	return ret_bool;
}


void ManipulatorState::printState() const {
	std::cout<<"End Effector is in location: "<<location_labels[state_space[0]]<<"\n";
	for (int i=1; i<state_space.size(); i++){
		if (state_space[i]==UNDEF){
			std::cout<<"Object: "<<obj_labels[i-1]<<"   has no location defined\n"; 
		} else {
			std::cout<<"Object: "<<obj_labels[i-1]<<"   is in location: "<<location_labels[state_space[i]]<<"\n";
		}
	}
}

bool ManipulatorState::isEqual(const ManipulatorState* compare_state_ptr) const {
	return (state_space == compare_state_ptr->getState());
}



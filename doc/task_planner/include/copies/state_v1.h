#ifndef STATE_H
#define STATE_H
#include<vector>
#include<iostream>
#include<unordered_map>
class State {
	protected:
		static std::vector<std::string> location_labels;
		//std::vector<std::string> location_labels;

		// State: index: obj_label_ind, location_ind>
		std::vector<int> state_space;
		static bool labels_set;
		int label2ind(std::string label, const std::vector<std::string>& labels) const;
	public:
		static const int UNDEF;
		const std::string UNDEF_label = "UNDEF";
		void setLocationLabels(const std::vector<std::string>& location_labels_);
		int returnNumLocations() const;
		std::string returnLocationLabel(int location_label_ind);
		bool isDefined() const;
		void setState(std::vector<int> state_);
		std::vector<int> getState() const;
		void copyTo(State* copy_state) const;
		void copyFrom(const State* copy_state);
};

#endif

#ifndef OBJSTATE_H
#define OBJSTATE_H

class ManipulatorState : public State {
	private:
		static unsigned int N_obj;
		static std::vector<std::string> obj_labels;
	public: 
		std::string returnEELocationLabel() const;
		int returnEELocation() const;
		bool isGrabbing(std::string eef_flag);
		bool isGrabbing(std::string eef_flag, int& grabbing_obj_ind);
		void setEELocation(std::string location_label);
		void setEELocation(int location_label_ind);
		void setObjLocationToEELocation(int obj_label_ind);
		void setObjLabels(const std::vector<std::string>& obj_labels_);
		std::string returnObjLocation(std::string obj_label) const;
		int returnObjLocation(int obj_label_ind) const;
		void setObjLocation(std::string obj_label, std::string location_label);
		void setObjLocation(int obj_label_ind, int location_label_ind);
		bool isOccupied(std::string location_label) const;
		bool isOccupied(int location_label_ind) const;
		void printState() const;
		bool isEqual(const ManipulatorState* compare_state_ptr) const;
};
				
#endif

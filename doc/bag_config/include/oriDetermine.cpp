#include<iostream>
#include<vector>
#include<utility>
#include<cmath>
#include "oriDetermine.h"
#include "matrix.h"


std::vector<std::pair<double[3], int>> bag;
std::vector<std::pair<double[3], int>> coords_3Ddata;
bool has_data;
int Niter;
double alpha;

oriDetermine::oriDetermine(std::vector<std::pair<double[3], int>>& bag_){
	bag.resize(bag_.size());
	for (int i=0; i<bag_.size(); i++){
		for (int ii=0; ii<3; ii++){
			bag[i].first[ii] = bag_[i].first[ii];
		}
		bag[i].second = bag_[i].second;
	}	
	has_data = false;
	Niter = 30;
	alpha = .04;
}

void oriDetermine::setData(std::vector<std::pair<double[3], int>>& coords_3Ddata_){
	if (has_data){
		coords_3Ddata.clear();
	}
	coords_3Ddata.resize(coords_3Ddata_.size());
	for (int i=0; i<coords_3Ddata_.size(); i++){
		for (int ii=0; ii<3; ii++){
			coords_3Ddata[i].first[ii] = coords_3Ddata_[i].first[ii];
		}
		coords_3Ddata[i].second = coords_3Ddata_[i].second;
	}	
	has_data = true;
}

void oriDetermine::setIterations(int Niter_){
	Niter = Niter_;
}

void oriDetermine::setAlpha(double alpha_){
	alpha = alpha_;
}

double oriDetermine::norm3(double* arr){
	double norm = 0.0;
	for (int i=0; i<3; i++){
		norm += (arr[i])*(arr[i]);
	}
	norm = std::sqrt(norm);
	return norm;
}

void oriDetermine::cross3(double* A, double* B, double* product){
	product[0] = A[1]*B[2] - B[1]*A[2];
	product[1] = -A[0]*B[2] + B[0]*A[2];
	product[2] = A[0]*B[1] - B[0]*A[1];

}

matrix<double> oriDetermine::q2R(double* v, double theta){
	double qr, qi, qj, qk;
	qr = std::cos(theta/2.0);
	qi = std::sin(theta/2.0)*v[0];
	qj = std::sin(theta/2.0)*v[1];
	qk = std::sin(theta/2.0)*v[2];
	double s = 1.0/(qr*qr + qi*qi + qj*qj + qk*qk);

	matrix<double> R(3,3);
	R(0,0) = 1.0 - 2.0*s*(qj*qj + qk*qk);
	R(0,1) = 2.0*s*(qi*qj - qk*qr); 
	R(0,2) = 2.0*s*(qi*qk + qj*qr); 
	R(1,0) = 2.0*s*(qi*qj + qk*qr); 
	R(1,1) = 1.0 - 2.0*s*(qi*qi + qk*qk); 
	R(1,2) = 2.0*s*(qj*qk - qi*qr); 
	R(2,0) = 2.0*s*(qi*qk - qj*qr); 
	R(2,1) = 2.0*s*(qj*qk + qi*qr); 
	R(2,2) = 1.0 - 2.0*s*(qi*qi + qj*qj); 
	return R;
}



matrix<double> oriDetermine::returnQ(){
	if (has_data){
		matrix<double> Q(3,3);
		Q.fill(0.0);
		int data_i = 0;
		double data_if = 0.0;
		std::vector<std::pair<double[3], int>> data_origin;
		std::vector<std::pair<double[3], int>> bag_origin;
		data_origin.resize(coords_3Ddata.size()-1);
		bag_origin.resize(bag.size()-1);
		matrix<double> Q_i(3,3);
		for (auto& data : coords_3Ddata){
			Q_i.eye();
			int color = data.second;
			int a = 0;
			for (int i=0; i<coords_3Ddata.size(); i++){

				if (i!=data_i){
					for (int ii=0; ii<3; ii++){
						data_origin[a].first[ii] = coords_3Ddata[i].first[ii] - data.first[ii];
					}
					data_origin[a].second = coords_3Ddata[i].second;
					a++;
				}

			}	
			a = 0;
			int i_currmark;
			for (int i=0; i<bag.size(); i++){
				if (bag[i].second == color){
					i_currmark = i;
					break;
				}		
			}
			for (int i=0; i<bag.size(); i++){
				if (i!=i_currmark){
					for (int ii=0; ii<3; ii++){
						bag_origin[a].first[ii] = bag[i].first[ii] - bag[i_currmark].first[ii];
					}
					bag_origin[a].second = bag[i].second;
					a++;
				}
			}
			for (int i=0; i<Niter; i++){
				double T[3] = {0.0, 0.0, 0.0};
				for (auto& datum : data_origin){
					int color2 = datum.second;
					int i_matchmark;
					for (int ii=0; ii<bag_origin.size(); ii++){
						if (bag_origin[ii].second == color2){
							i_matchmark = ii;
							break;
						}		
					}
					double force[3] = {0.0, 0.0, 0.0};
					double product[3] = {0.0, 0.0, 0.0};
					double r[3] = {0.0, 0.0, 0.0};
					double norm_r;
					norm_r = norm3(bag_origin[i_matchmark].first);	
					for (int ii=0; ii<3; ii++){
						force[ii] = datum.first[ii] - bag_origin[i_matchmark].first[ii];
						
						r[ii] = bag_origin[i_matchmark].first[ii]/norm_r;	 

					}
					cross3(r, force, product);
					for (int ii=0; ii<3; ii++){
						T[ii] = T[ii] + product[ii];
					}
				}
				double v[3];
				double norm_T = norm3(T);
				for (int ii=0; ii<3; ii++){
					v[ii] = T[ii]/norm_T;	
				}
				double theta = norm_T*alpha;
				matrix<double> R(3,3);
				R = q2R(v, theta);
				for (auto& bag_org_i : bag_origin){
					double temp_r[3];
					for (int ii=0; ii<3; ii++){
						double sum = 0.0;
						for (int iii=0; iii<3; iii++){
							sum += R(ii,iii)*bag_org_i.first[iii];
						}
						temp_r[ii] = sum;
					}
					for (int ii=0; ii<3; ii++){
						bag_org_i.first[ii] = temp_r[ii];
					}	
				}
				Q_i = R*Q_i;
			}
			Q.scale(data_if);
			Q = Q + Q_i;
			double scaler = 1.0/(data_if + 1.0);
			Q.scale(scaler);
			data_i++;
			data_if = data_if + 1.0;
		}	
		return Q;
	} else {
		std::cout<<"Error: Must input data set before calling 'returnQ'"<<std::endl;
	}
}

#include<iostream>
#include<vector>
#include "matrix.h"


template <class T>
matrix<T>::matrix(unsigned int r_, unsigned int c_) : rows(r_), cols(c_){
	matrix::mat.resize(rows);
	for (int i=0; i<rows; i++){
		mat[i].resize(cols);	
	}
}

template <class T>
void matrix<T>::set(std::vector<std::vector<T>>& set_vec){
	matrix::mat = set_vec;
}

template <class T>
void matrix<T>::fill(T elem){
	for (int i=0; i<rows; i++){
		for (int j=0; j<cols; j++){
			mat[i][j] = elem;
		}
	} 
}

template <class T>
void matrix<T>::eye(){
	if (rows==cols) {
		T uno = 1.0;
		T zero = 0.0;
		for (int i=0; i<rows; i++){
			for (int j=0; j<cols; j++){
				if (j==i){
					mat[i][j] = uno;
				} else {
					mat[i][j] = zero;
				}
			}
		} 

	} else {
		std::cout<<"Error: Matrix is not square, cannot make identity\n";
	}
}


template <class T>
void matrix<T>::scale(T scalar){
	for (int i=0; i<mat.size(); i++){
		for (int j=0; j<mat[i].size(); j++){
			mat[i][j] = scalar*mat[i][j];
		}
	}

}

template <class T>
void matrix<T>::add(T scalar){
	for (int i=0; i<mat.size(); i++){
		for (int j=0; j<mat[i].size(); j++){
			mat[i][j] = scalar +  mat[i][j];
		}
	}

}

template <class T>
void matrix<T>::print() const {
	for (int i=0; i<mat.size(); i++){
		std::cout<<"Row "<<i<<": ";
		for (int j=0; j<mat[i].size(); j++){
			std::cout<<mat[i][j]<<"   ";
		}
		std::cout<<"\n";
	}
}


/// operators ///

//set
template <class T>
T& matrix<T>::operator() (int i, int j){
	return mat[i][j];
}

//get
template <class T>
T matrix<T>::operator() (int i, int j) const {
	return mat[i][j];
}

template <class T>
matrix<T> matrix<T>::operator* (const matrix<T>& arg) const{
	if (arg.rows==cols){
		matrix<T> ret_mat(rows, arg.cols);
		for (int i=0; i<rows; i++){
			for (int j=0; j<arg.cols; j++){
				T sum = 0;
				for (int ii=0; ii<cols; ii++){
					sum += mat[i][ii]*arg(ii,j);
				}
				ret_mat(i,j) = sum;
			}
		}
		return ret_mat;

	} else {
		std::cout<<"Error: Cannot multiply matrices: inner dimension mismatch\n";
	}
}


template <class  T>
void matrix<T>::operator= (const matrix<T>& arg) {
	if (arg.rows==rows&&arg.cols==cols){
		for (int i=0; i<arg.rows; i++){
			for (int j=0; j<arg.cols; j++){
				mat[i][j] = arg(i,j);
			}
		}
	} else {
		std::cout<<"Error: Dimensions on either side of '=' don't match\n";
	}
}

template <class T>
matrix<T> matrix<T>::operator+ (const matrix<T>& arg) const{
	if (arg.rows==cols&&arg.cols==cols){
		matrix<T> ret_mat(rows, cols);
		for (int i=0; i<rows; i++){
			for (int j=0; j<cols; j++){
				ret_mat(i,j) = mat[i][j] + arg(i,j);
			}
		}
		return ret_mat;

	} else {
		std::cout<<"Error: Cannot add matrices: dimension mismatch\n";
	}
}

template <class T>
matrix<T> matrix<T>::operator- (const matrix<T>& arg) const{
	if (arg.rows==cols&&arg.cols==cols){
		matrix<T> ret_mat(rows, cols);
		for (int i=0; i<rows; i++){
			for (int j=0; j<cols; j++){
				ret_mat(i,j) = mat[i][j] - arg(i,j);
			}
		}
		return ret_mat;

	} else {
		std::cout<<"Error: Cannot add matrices: dimension mismatch\n";
	}
}


template class matrix<double>;
template class matrix<int>;
//template class matrix<float>;


#pragma once
#include<vector>

template <class T>
class matrix {
	private:
		std::vector<std::vector<T>> mat;
	public:
		matrix(unsigned int r_, unsigned int c_);
		const unsigned int rows, cols;
		void set(std::vector<std::vector<T>>& set_vec);
		void fill(T elem);
		void eye();
		void scale(T scalar);
		void add(T scalar);
		void print() const;
		T& operator() (int i, int j);
		T operator() (int i, int j) const;
		matrix<T> operator* (const matrix<T>& arg) const;
		void operator= (const matrix<T>& arg);
		matrix<T> operator+ (const matrix<T>& arg) const;
		matrix<T> operator- (const matrix<T>& arg) const;
};

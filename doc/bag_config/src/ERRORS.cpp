#include "ERRORS.hpp"
using namespace std;

void WARNING(string str)
{
	cout << "\033[1;93m" << str << "\033[0m" << endl;
}


void ERROR(string str)
{
	cout << "\033[1;91m" << str << "\033[0m" << endl;
	exit(EXIT_FAILURE);
}
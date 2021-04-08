#include "ERRORS.hpp"
using namespace std;

void WARNING(string str)
{
	cout << "\033[1;93m" << "WARNING: " << str << "\033[0m" << endl;
}


void ERROR(string str)
{
	// cout << "\033[1;91m" << str << "\033[0m" << endl;
	ROS_FATAL("%s", str.c_str());
	cout << "\033[1;91m" << "It is recommended that you terminate the program  with cntrl-C now to troubleshoot." << "\033[0m" << endl;
	cin.get();
	// exit(EXIT_FAILURE);
	// throw std::exception();
}


bool CheckParam(string param_str, int severity)
{
	stringstream warn_str;

	if(!ros::param::has(param_str))
	{
		warn_str << "The rosparam " << param_str << " does not exist.";
		if(severity == 1) // warning
		{
			WARNING(warn_str.str());
		}
		else if(severity == 2) // error
		{
			ERROR(warn_str.str());
		}
		return false;
	}
	return true;
}


bool CheckParam(string param_str, int severity, double dflt)
{
	stringstream warn_str;
	stringstream warn_str_tmp;

	if(!ros::param::has(param_str))
	{
		warn_str_tmp << "The rosparam " << param_str << " does not exist.";
		if(severity == 1) // warning
		{
			warn_str << warn_str_tmp.str() << " Defaulting " << param_str << " to " << dflt << ".";
			WARNING(warn_str.str());
		}
		else if(severity == 2) // error
		{
			ERROR(warn_str.str());
		}
		return false;
	}
	return true;
}


bool CheckParam(string param_str, int severity, string dflt)
{
	stringstream warn_str;
	stringstream warn_str_tmp;

	if(!ros::param::has(param_str))
	{
		warn_str_tmp << "The rosparam " << param_str << " does not exist.";
		if(severity == 1) // warning
		{
			warn_str << warn_str_tmp.str() << " Defaulting " << param_str << " to " << dflt << ".";
			WARNING(warn_str.str());
		}
		else if(severity == 2) // error
		{
			ERROR(warn_str.str());
		}
		return false;
	}
	return true;
}


bool CheckParam(string param_str, int severity, bool dflt)
{
	stringstream warn_str;
	stringstream warn_str_tmp;

	if(!ros::param::has(param_str))
	{
		warn_str_tmp << "The rosparam " << param_str << " does not exist.";
		if(severity == 1) // warning
		{
			warn_str << warn_str_tmp.str() << " Defaulting " << param_str << " to " << dflt << ".";
			WARNING(warn_str.str());
		}
		else if(severity == 2) // error
		{
			ERROR(warn_str.str());
		}
		return false;
	}
	return true;
}


bool CheckParamSize(string param_str, int size)
{
	vector<double> param;
	ros::param::get(param_str, param);
	stringstream warn_str;

	if(param.size() != size)
	{
		warn_str << "The rosparam " + param_str + " is the incorrect size.";
		ERROR(warn_str.str());
		return false;
	}
	return true;
}
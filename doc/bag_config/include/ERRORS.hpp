#ifndef ERRORS_H
#define ERRORS_H

#include <iostream>
#include <string>
#include <sstream>
#include "ros/ros.h"
#include "ros/console.h"

using namespace std;

void WARNING(string str);
void ERROR(string str);
bool CheckParam(string param_str, int severity);
bool CheckParam(string param_str, int severity, double dflt);
bool CheckParam(string param_str, int severity, string dflt);
bool CheckParam(string param_str, int severity, bool dflt);
bool CheckParamSize(string param_str, int size);

#endif
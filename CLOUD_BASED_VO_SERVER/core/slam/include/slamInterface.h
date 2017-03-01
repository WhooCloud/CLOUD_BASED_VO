#ifndef SLAMINTERFACE_H
#define SLAMINTERFACE_H

#include <iostream>
#include <string>
#include <string.h>

using namespace std;

extern "C" char* FFIInterface(const char* data);
string processData(const char* data);

#endif

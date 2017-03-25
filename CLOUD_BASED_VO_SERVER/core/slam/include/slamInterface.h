#ifndef SLAMINTERFACE_H
#define SLAMINTERFACE_H

#include <iostream>
#include <string>
#include <string.h>
#include <memory>

using namespace std;

extern "C" char* FFIInterface(const char* data);
extern "C" char* FFICompressString(const char* data);
extern "C" char* FFIDecompressString(const char* data);
string processData(const char* data);

#endif

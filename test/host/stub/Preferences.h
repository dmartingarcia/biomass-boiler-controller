#pragma once
#include <map>
#include <string>
class Preferences{ public: static std::map<std::string,int>& db(){static std::map<std::string,int> m;return m;}
 bool begin(const char*,bool){return true;} void end(){} bool isKey(const char*k){return db().count(k);}
 int8_t getChar(const char*k,int8_t d){return db().count(k)?db()[k]:d;} void putChar(const char*k,int8_t v){db()[k]=v;} void putUChar(const char*k,uint8_t v){db()[k]=v;} void clear(){db().clear();} };

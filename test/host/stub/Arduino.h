#pragma once
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <algorithm>
extern unsigned long fakeMs;
inline unsigned long millis(){return fakeMs;}
class Print{public:
 virtual size_t write(const uint8_t*b,size_t n){fwrite(b,1,n,stdout);return n;}
 size_t print(const char*s){return write((const uint8_t*)s,strlen(s));}
 size_t print(char c){return write((const uint8_t*)&c,1);}
 size_t print(int v){char t[16];int n=snprintf(t,16,"%d",v);return write((const uint8_t*)t,n);}
 size_t print(unsigned long v){char t[24];int n=snprintf(t,24,"%lu",v);return write((const uint8_t*)t,n);}
};

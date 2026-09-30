#pragma once
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include <cassert>
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define OUTPUT 1
#define LOW 0
#define HIGH 1
inline unsigned long nowMs=0;
inline int pins[32]={};
inline std::vector<int> events;
inline unsigned long millis(){return nowMs;}
inline void pinMode(int,int){}
inline void digitalWrite(int p,int v){pins[p]=v; events.push_back(p);}
inline void analogWrite(int p,int v){pins[p]=v; events.push_back(p);}
inline int analogRead(int){return 0;}
struct FakeSerial {
 bool connected=true; int room=512; int writes=0; std::string input, output;
 void print(const char* v){output+=v;}
 void print(int v){output+=std::to_string(v);}
 void print(float v,int digits){char b[64];snprintf(b,sizeof(b),"%.*f",digits,v);output+=b;}
 void println(int v){print(v);output+="\n";}
 void println(float v,int digits){print(v,digits);output+="\n";}
 void begin(long){}
 operator bool(){return connected;}
 int available(){return input.size();}
 int read(){int c=input[0];input.erase(0,1);return c;}
 int availableForWrite(){return room;}
 size_t write(const uint8_t* p,size_t n){assert(n<=(size_t)room);writes++;output.append((const char*)p,n);return n;}
};
inline FakeSerial Serial;

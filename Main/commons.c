#include "commons.h"
//insert a into [b,c]
//#define constrain(a,b,c) ((a)<(b)?(b):(a)>(c)?c:a)
int constrain(int a, int b, int c){
	return ((a)<(b)?(b):(a)>(c)?c:a);
}
int dead_zone(int a, int b){
	return ((a)>(b)?(a):(a)<(-b)?(a):0);
}
float constrain_f(float a, float b, float c){
	return ((a)<(b)?(b):(a)>(c)?c:a);
}
int minimum(int a, int b){
	return (a>b?b:a);
}
int maximum(int a, int b){
	return (a>b?a:b);
}

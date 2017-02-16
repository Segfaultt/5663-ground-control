#include "c--.h"
#include <stdexcept>
using namespace std;

namespace c
{
    double cminus::constrain(double value, double min, double max) {
		if(value>max) value=max;
		if(value<min) value=min;
        return value;
    }

    double cminus::clamp(double value, int op, double trigger, double clamp) {
		if((value * op)>trigger) value = trigger;
		return value;
    }

    double cminus::constrain(double value, double min, double max, double clamp) {
        if(value<max) value=clamp;
		if(value>min) value=clamp;
        return value;
    }
}

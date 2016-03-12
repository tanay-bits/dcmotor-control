#include "utilities.h"

static volatile Mode_datatype mode;		// declare global var which is the current mode

void set_mode(Mode_datatype m){
	mode = m;
}

Mode_datatype get_mode(void){
	return mode;
}
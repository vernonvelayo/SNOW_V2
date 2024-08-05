#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <sys/types.h>
#include <sys/ioctl.h>



#include "kbhit.h"
#include "motor.h"
#include "joint.h"
#include "utility.h"
#include "eso.h"

                      //         0-> 1-> 2-> 3-> 4-> 5 
ESO_PSI esoPsi(-120,35);// (b,wo) 10->20->25->30->40->45
ESO_THETA_ACCEL esoThetaAccel(1,20);

ESO_THETA_ACCEL esoJ0(100,10);
ESO_THETA_ACCEL esoJ1(100,10);


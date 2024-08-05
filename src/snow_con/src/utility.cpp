#include <stdio.h>
#include <stdlib.h>
#include <curses.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <sys/types.h>
#include <sys/ioctl.h>

#include "kbhit.h"
#include "motor.h"
#include "joint.h"


double sign(double value)
{

  if( value > 0.0 )
    return 1.0;
  else if( value < 0.0 )
    return -1.0;
  else
    return 0.0;
}

double deadzone(double value, double threshold)
{
  double retval;

  if ( fabs(value) >= threshold ) 
    retval = value - sign(value)*threshold;
  else
    retval = 0.0;

  return retval;
}

double saturate(double value,double threshold)
{

  double retval;

  if ( value >= threshold ) 
    retval = threshold;
  else if( value <= -threshold )
    retval = -threshold;
  else
    retval = value;

  return retval;
}

double saturateNonSym(double value,double thHigh, double thLow)
{

  double retval;

  if ( value >= thHigh ) 
    retval = thHigh;
  else if( value <= thLow )
    retval = thLow;
  else
    retval = value;

  return retval;
}



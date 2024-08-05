#ifndef __UTILITY__

#define __UTILITY__

#include <new>
#include "upentar_consts.h"



class ROBUST_DIFF{
  private:

    short init;

    double xHat;
    double x;
    double l[2];

  public:

    double w0;
    double dXHat;

    ROBUST_DIFF(double w0I):w0(w0I)
    {
      init = 0;
      calculateGain();
    }

    ROBUST_DIFF()
    {
      init = 0;
      w0 = 10;
      calculateGain();
    }

    void calculateGain()
    {
      l[0] = 2*w0;
      l[1] = w0*w0;
    }

    double calculate(double in)
    {
      double e;

      if (init == 0) {
        init = 1;
        xHat = x;
      } 

      calculateGain();

      x = in;
      e = x - xHat;

      xHat = xHat + dt*(dXHat + l[0]*e);
      dXHat = dXHat + dt*(l[1]*e);

      return dXHat;
    };
};


class TIME_SERIES{

  private:

    double *xVec;
    short now;
    short bufferLength;
    unsigned long updateStamp;


    short n(short i)
    {
      return (now - i + bufferLength)%bufferLength;
    }

    short createBuffer(short length)
    {

      xVec = new double [length];

      if (xVec == 0)
        return 0;
      else
        return 1;

    }

    short destroyBuffer()
    {

      delete[] xVec;

      return 1;
    }

    void incNow()
    {
      now++;
      now%=bufferLength;
    }


  public:

    double x(short i,double val)
    {
      if (i == 0) {
        incNow();
      }

      return (xVec[n(i)] = val);
    };
    double x(short i) {return (xVec[n(i)]); };

    TIME_SERIES()
    {
      bufferLength = 3;
      createBuffer(bufferLength);
    }

    TIME_SERIES(int n)
    {
      bufferLength = n;
      createBuffer(bufferLength);
    }

    ~TIME_SERIES()
    {
      destroyBuffer();
    }

}; 



double sign(double value);
double deadzone(double value, double threshold);
double saturate(double value,double threshold);
double saturateNonSym(double value,double thHigh, double thLow);

#endif

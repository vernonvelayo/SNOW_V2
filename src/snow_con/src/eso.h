#ifndef __ESO__
#define __ESO__

#include "utility.h"
#include "upentar_consts.h"
#include<math.h>

class ESO {
  private:
    double L[3];//observer gain
    double h;//time derivative of f

    ROBUST_DIFF fHatDiff;


  public:
    double b;//input coefficient
    double wo;//observer bandwidth
    double q;//position
    double dq;//velocity
    double f;//function value
    double qHat;//position estimate
    double dqHat;//velocity estimate
    double fHat;//function estimate
    double dFHat;//function derivative estimate
    double dFHatFilter;//function derivative filtered estimate 
    double e;//error 
    double u;//input
    double dU;//input
    double uPrev;//previous value of input

    short init;
    short hCalcOn;

    ESO(
        double bConstructor,
        double woConstructor
       ):
      b(bConstructor),
      wo(woConstructor)
  {
    q = 0.0;
    dq = 0.0;

    f = 0.0;
    qHat = 0.0;
    dqHat = 0.0;
    fHat = 0.0;
    dFHat = 0.0;
    dFHatFilter = 0.0;
    e = 0.0;
    u = 0.0;
    dU = 0.0;
    uPrev = 0.0;
    init = 0;
    hCalcOn = 0;
  }

    virtual double eCalc(void)
    {
      return (q-qHat);
    }

    virtual double hCalc(void)
    {
      return 0;
    }

    virtual void gainCalculate(void)
    {
      L[0] = 3*wo;
      L[1] = 3*wo*wo;
      L[2] = wo*wo*wo;
    };

    virtual double dFHatCalc()
    {
      dFHat = fHatDiff.calculate(fHat);

      return dFHat;
    }

    virtual void predict(double qM, double dqM, double uM)
    {

      q = qM;
      dq = dqM;
      u = uM;

      gainCalculate();

      if( init == 0 ){
        init = 1;
        qHat = q;
        dqHat = dq;
        dU = 0.0;
      }

      h = hCalc();

      e = eCalc();

      qHat  = qHat  + dt*(dqHat + L[0]*e);
      dqHat = dqHat + dt*(fHat + L[1]*e + b*u );
      fHat = fHat + dt*(h + L[2]*e);

      dFHatCalc();
    }

    virtual double dUCalc()
    {

      dU = (u - uPrev)/dt;

      uPrev = u;

      return dU;
    };


    void setFHatDiffGain(double wNew)
    {
      fHatDiff.w0 = wNew;
    }

};

class ESO_THETA_ACCEL: public ESO{

  private:

  public:

    using ESO::ESO;

    double hCalc(void)
    {
      return 0;
    }
};

class ESO_PSI: public ESO{

  private:
    ROBUST_DIFF ddqHat;

  public:

    using ESO::ESO;

    double hCalc(void)
    {
      ddqHat.w0 = 15; 
      ddqHat.calculate(dq);

      return 0;
    }

    double eCalc(void)
    {
      double eLocal;
    
      eLocal =(q-qHat); 
/*
      if( fabs(eLocal) < 0.0001){
        eLocal = 0.0;
      }
*/

      return eLocal;
    }

};

class ESO_PSI_BAR: public ESO{

  private:
    double softStart;

  public:

    ESO *esoThetaBarP;

    short hOn;
  
    double hCalc(void)
    {
       return 0;
    }

};

class ESO_THETA_BAR: public ESO{

  public:

    ESO *esoPsiBarP;

    double hCalc(void)
    {
      return (0.0*esoPsiBarP->dq + b*esoPsiBarP->dU);
    }

    double eCalc(void)
    {
      return saturate( (q-qHat), 0.08);
    }
};


//extern ESO_PSI esoPsi;
//extern ESO_PSI_BAR esoPsiBar;
//extern ESO_THETA_BAR esoThetaBar;



#endif

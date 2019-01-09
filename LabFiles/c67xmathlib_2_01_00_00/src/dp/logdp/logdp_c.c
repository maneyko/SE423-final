/* ======================================================================= */
/* log2dp_c.h - double precision floating point ln                         */
/*             optimized  C implementation (w/ intrinsics) single sample   */
/*                                                                         */
/* Rev 0.0.1                                                               */
/*                                                                         */
/* ----------------------------------------------------------------------- */
/*            Copyright (c) 2010 Texas Instruments, Incorporated.          */
/*                           All Rights Reserved.                          */
/* ======================================================================= */


#include "logdp_c.h"

static inline double divdpMod_clog(double a, double b)
{


/*                                      */
double  X, TWO=2.0;



        X = _rcpdp(b) ;
        
        X = X*( TWO - b*X ) ;
        X = X*( TWO - b*X ) ;
		X = X*( TWO - b*X );
        
        
		// a*X = a*(1/b)
		X = a*X;



        return (X) ;           /* returns DP FP value = 1.0 / A */

}



double logdp_c(double a)
{


// double One=1.0 ;
double Zero = 0.0, Half = 0.5 ;
double srHalf= 0.70710678118654752440 ;         /*  sqrt(0.5)   */
double MIN   =    2.2250738585072014e-308 ;
double LnMin = -708.3964185322641 ;
double MAX   =    1.7976931348623157e+308 ;
double LnMax =  709.7827138470578 ;
double  a0 =-0.64124943423745581147e+2 ;
double  a1 = 0.16383943563021534222e+2 ;
double  a2 =-0.78956112887491257267e+0 ;
double  b0 =-0.76949932108494879777e+3 ;
double  b1 = 0.31203222091924532844e+3 ;
double  b2 =-0.35667977739034646171e+2 ; /*  Note b3 = 1.0        */
double  c1 = 0.693359375  ;       		 /*  355/512      */
double  c2 =-2.121944400546905827679e-4 ;

double  Y,Z,zn,zd,X,W,Rz,Sa,Bd,Cn,Da ;
int     N, exp, upper; 

        Y = a ;
        exp = _extu(_hi(Y),1,21);
       	N = exp - 1022;

 		upper = _clr(_hi(Y),20,31);
		upper = 0x3fe00000 | upper;
		Z = _itod(upper, _lo(Y));
		if (exp == 0) Z = 0;


        if (Z > srHalf)
                { zn=(Z-Half)-Half ; zd=Z*Half+Half  ; }
        else
                { zn=Z-Half; zd=zn*Half+Half ; N=N-1 ; }
        
        
        //X = zn/zd;
        X = divdpMod_clog(zn,zd);
        
        W = X*X ;
        Bd= ( (W+b2) * W + b1) * W + b0 ;
        Cn= (W*a2 + a1) * W + a0 ;
        Rz= W * divdpMod_clog(Cn,Bd);//Cn / Bd ;
        Sa = X + X*Rz ;
        Cn= N ;
        Da = (Cn*c2 + Sa) + Cn*c1 ;



		if (Y <= Zero) Da = 0;//{Z = setnmx(Y) ; Da = (Z) ; }
        if (Y < MIN)  Da = (LnMin)  ;
        if (Y > MAX)  Da = (LnMax)  ;




        return (Da) ;

}

/* ======================================================================== */
/*  End of file: log2dp_c.c                                                 */
/* ------------------------------------------------------------------------ */
/*          Copyright (C) 2010 Texas Instruments, Incorporated.             */
/*                          All Rights Reserved.                            */
/* ======================================================================== */

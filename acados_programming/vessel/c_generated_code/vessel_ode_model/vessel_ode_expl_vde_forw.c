/* This file was automatically generated by CasADi 3.6.3.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) vessel_ode_expl_vde_forw_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

static const casadi_int casadi_s0[10] = {6, 1, 0, 6, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s1[45] = {6, 6, 0, 6, 12, 18, 24, 30, 36, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s2[24] = {6, 3, 0, 6, 12, 18, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s3[7] = {3, 1, 0, 3, 0, 1, 2};
static const casadi_int casadi_s4[3] = {0, 0, 0};

/* vessel_ode_expl_vde_forw:(i0[6],i1[6x6],i2[6x3],i3[3],i4[])->(o0[6],o1[6x6],o2[6x3]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[0]? arg[0][2] : 0;
  a1=cos(a0);
  a2=arg[0]? arg[0][3] : 0;
  a3=(a1*a2);
  a4=sin(a0);
  a5=arg[0]? arg[0][4] : 0;
  a6=(a4*a5);
  a3=(a3-a6);
  if (res[0]!=0) res[0][0]=a3;
  a3=sin(a0);
  a6=(a3*a2);
  a7=cos(a0);
  a8=(a7*a5);
  a6=(a6+a8);
  if (res[0]!=0) res[0][1]=a6;
  a6=arg[0]? arg[0][5] : 0;
  if (res[0]!=0) res[0][2]=a6;
  a8=-3.0283716283716284e+00;
  a9=(a8*a2);
  a10=1.9980019980019980e-02;
  a11=arg[3]? arg[3][0] : 0;
  a11=(a10*a11);
  a9=(a9+a11);
  if (res[0]!=0) res[0][3]=a9;
  a9=-1.5706495969653864e+00;
  a11=(a9*a5);
  a12=1.1853959222380275e-02;
  a13=arg[3]? arg[3][1] : 0;
  a13=(a12*a13);
  a11=(a11+a13);
  if (res[0]!=0) res[0][4]=a11;
  a11=-2.0046484601975592e+00;
  a6=(a11*a6);
  a13=5.8105752469494475e-02;
  a14=arg[3]? arg[3][2] : 0;
  a14=(a13*a14);
  a6=(a6+a14);
  if (res[0]!=0) res[0][5]=a6;
  a6=arg[1]? arg[1][3] : 0;
  a14=(a1*a6);
  a15=sin(a0);
  a16=arg[1]? arg[1][2] : 0;
  a17=(a15*a16);
  a17=(a2*a17);
  a14=(a14-a17);
  a17=cos(a0);
  a18=(a17*a16);
  a18=(a5*a18);
  a19=arg[1]? arg[1][4] : 0;
  a20=(a4*a19);
  a18=(a18+a20);
  a14=(a14-a18);
  if (res[1]!=0) res[1][0]=a14;
  a14=cos(a0);
  a18=(a14*a16);
  a18=(a2*a18);
  a20=(a3*a6);
  a18=(a18+a20);
  a20=(a7*a19);
  a21=sin(a0);
  a16=(a21*a16);
  a16=(a5*a16);
  a20=(a20-a16);
  a18=(a18+a20);
  if (res[1]!=0) res[1][1]=a18;
  a18=arg[1]? arg[1][5] : 0;
  if (res[1]!=0) res[1][2]=a18;
  a6=(a8*a6);
  if (res[1]!=0) res[1][3]=a6;
  a19=(a9*a19);
  if (res[1]!=0) res[1][4]=a19;
  a18=(a11*a18);
  if (res[1]!=0) res[1][5]=a18;
  a18=arg[1]? arg[1][9] : 0;
  a19=(a1*a18);
  a6=arg[1]? arg[1][8] : 0;
  a20=(a15*a6);
  a20=(a2*a20);
  a19=(a19-a20);
  a20=(a17*a6);
  a20=(a5*a20);
  a16=arg[1]? arg[1][10] : 0;
  a22=(a4*a16);
  a20=(a20+a22);
  a19=(a19-a20);
  if (res[1]!=0) res[1][6]=a19;
  a19=(a14*a6);
  a19=(a2*a19);
  a20=(a3*a18);
  a19=(a19+a20);
  a20=(a7*a16);
  a6=(a21*a6);
  a6=(a5*a6);
  a20=(a20-a6);
  a19=(a19+a20);
  if (res[1]!=0) res[1][7]=a19;
  a19=arg[1]? arg[1][11] : 0;
  if (res[1]!=0) res[1][8]=a19;
  a18=(a8*a18);
  if (res[1]!=0) res[1][9]=a18;
  a16=(a9*a16);
  if (res[1]!=0) res[1][10]=a16;
  a19=(a11*a19);
  if (res[1]!=0) res[1][11]=a19;
  a19=arg[1]? arg[1][15] : 0;
  a16=(a1*a19);
  a18=arg[1]? arg[1][14] : 0;
  a20=(a15*a18);
  a20=(a2*a20);
  a16=(a16-a20);
  a20=(a17*a18);
  a20=(a5*a20);
  a6=arg[1]? arg[1][16] : 0;
  a22=(a4*a6);
  a20=(a20+a22);
  a16=(a16-a20);
  if (res[1]!=0) res[1][12]=a16;
  a16=(a14*a18);
  a16=(a2*a16);
  a20=(a3*a19);
  a16=(a16+a20);
  a20=(a7*a6);
  a18=(a21*a18);
  a18=(a5*a18);
  a20=(a20-a18);
  a16=(a16+a20);
  if (res[1]!=0) res[1][13]=a16;
  a16=arg[1]? arg[1][17] : 0;
  if (res[1]!=0) res[1][14]=a16;
  a19=(a8*a19);
  if (res[1]!=0) res[1][15]=a19;
  a6=(a9*a6);
  if (res[1]!=0) res[1][16]=a6;
  a16=(a11*a16);
  if (res[1]!=0) res[1][17]=a16;
  a16=arg[1]? arg[1][21] : 0;
  a6=(a1*a16);
  a19=arg[1]? arg[1][20] : 0;
  a20=(a15*a19);
  a20=(a2*a20);
  a6=(a6-a20);
  a20=(a17*a19);
  a20=(a5*a20);
  a18=arg[1]? arg[1][22] : 0;
  a22=(a4*a18);
  a20=(a20+a22);
  a6=(a6-a20);
  if (res[1]!=0) res[1][18]=a6;
  a6=(a14*a19);
  a6=(a2*a6);
  a20=(a3*a16);
  a6=(a6+a20);
  a20=(a7*a18);
  a19=(a21*a19);
  a19=(a5*a19);
  a20=(a20-a19);
  a6=(a6+a20);
  if (res[1]!=0) res[1][19]=a6;
  a6=arg[1]? arg[1][23] : 0;
  if (res[1]!=0) res[1][20]=a6;
  a16=(a8*a16);
  if (res[1]!=0) res[1][21]=a16;
  a18=(a9*a18);
  if (res[1]!=0) res[1][22]=a18;
  a6=(a11*a6);
  if (res[1]!=0) res[1][23]=a6;
  a6=arg[1]? arg[1][27] : 0;
  a18=(a1*a6);
  a16=arg[1]? arg[1][26] : 0;
  a20=(a15*a16);
  a20=(a2*a20);
  a18=(a18-a20);
  a20=(a17*a16);
  a20=(a5*a20);
  a19=arg[1]? arg[1][28] : 0;
  a22=(a4*a19);
  a20=(a20+a22);
  a18=(a18-a20);
  if (res[1]!=0) res[1][24]=a18;
  a18=(a14*a16);
  a18=(a2*a18);
  a20=(a3*a6);
  a18=(a18+a20);
  a20=(a7*a19);
  a16=(a21*a16);
  a16=(a5*a16);
  a20=(a20-a16);
  a18=(a18+a20);
  if (res[1]!=0) res[1][25]=a18;
  a18=arg[1]? arg[1][29] : 0;
  if (res[1]!=0) res[1][26]=a18;
  a6=(a8*a6);
  if (res[1]!=0) res[1][27]=a6;
  a19=(a9*a19);
  if (res[1]!=0) res[1][28]=a19;
  a18=(a11*a18);
  if (res[1]!=0) res[1][29]=a18;
  a18=arg[1]? arg[1][33] : 0;
  a19=(a1*a18);
  a6=arg[1]? arg[1][32] : 0;
  a15=(a15*a6);
  a15=(a2*a15);
  a19=(a19-a15);
  a17=(a17*a6);
  a17=(a5*a17);
  a15=arg[1]? arg[1][34] : 0;
  a20=(a4*a15);
  a17=(a17+a20);
  a19=(a19-a17);
  if (res[1]!=0) res[1][30]=a19;
  a14=(a14*a6);
  a14=(a2*a14);
  a19=(a3*a18);
  a14=(a14+a19);
  a19=(a7*a15);
  a21=(a21*a6);
  a21=(a5*a21);
  a19=(a19-a21);
  a14=(a14+a19);
  if (res[1]!=0) res[1][31]=a14;
  a14=arg[1]? arg[1][35] : 0;
  if (res[1]!=0) res[1][32]=a14;
  a18=(a8*a18);
  if (res[1]!=0) res[1][33]=a18;
  a15=(a9*a15);
  if (res[1]!=0) res[1][34]=a15;
  a14=(a11*a14);
  if (res[1]!=0) res[1][35]=a14;
  a14=arg[2]? arg[2][3] : 0;
  a15=(a1*a14);
  a18=sin(a0);
  a19=arg[2]? arg[2][2] : 0;
  a21=(a18*a19);
  a21=(a2*a21);
  a15=(a15-a21);
  a21=cos(a0);
  a6=(a21*a19);
  a6=(a5*a6);
  a17=arg[2]? arg[2][4] : 0;
  a20=(a4*a17);
  a6=(a6+a20);
  a15=(a15-a6);
  if (res[2]!=0) res[2][0]=a15;
  a15=cos(a0);
  a6=(a15*a19);
  a6=(a2*a6);
  a20=(a3*a14);
  a6=(a6+a20);
  a20=(a7*a17);
  a0=sin(a0);
  a19=(a0*a19);
  a19=(a5*a19);
  a20=(a20-a19);
  a6=(a6+a20);
  if (res[2]!=0) res[2][1]=a6;
  a6=arg[2]? arg[2][5] : 0;
  if (res[2]!=0) res[2][2]=a6;
  a14=(a8*a14);
  a10=(a10+a14);
  if (res[2]!=0) res[2][3]=a10;
  a17=(a9*a17);
  if (res[2]!=0) res[2][4]=a17;
  a6=(a11*a6);
  if (res[2]!=0) res[2][5]=a6;
  a6=arg[2]? arg[2][9] : 0;
  a17=(a1*a6);
  a10=arg[2]? arg[2][8] : 0;
  a14=(a18*a10);
  a14=(a2*a14);
  a17=(a17-a14);
  a14=(a21*a10);
  a14=(a5*a14);
  a20=arg[2]? arg[2][10] : 0;
  a19=(a4*a20);
  a14=(a14+a19);
  a17=(a17-a14);
  if (res[2]!=0) res[2][6]=a17;
  a17=(a15*a10);
  a17=(a2*a17);
  a14=(a3*a6);
  a17=(a17+a14);
  a14=(a7*a20);
  a10=(a0*a10);
  a10=(a5*a10);
  a14=(a14-a10);
  a17=(a17+a14);
  if (res[2]!=0) res[2][7]=a17;
  a17=arg[2]? arg[2][11] : 0;
  if (res[2]!=0) res[2][8]=a17;
  a6=(a8*a6);
  if (res[2]!=0) res[2][9]=a6;
  a20=(a9*a20);
  a12=(a12+a20);
  if (res[2]!=0) res[2][10]=a12;
  a17=(a11*a17);
  if (res[2]!=0) res[2][11]=a17;
  a17=arg[2]? arg[2][15] : 0;
  a1=(a1*a17);
  a12=arg[2]? arg[2][14] : 0;
  a18=(a18*a12);
  a18=(a2*a18);
  a1=(a1-a18);
  a21=(a21*a12);
  a21=(a5*a21);
  a18=arg[2]? arg[2][16] : 0;
  a4=(a4*a18);
  a21=(a21+a4);
  a1=(a1-a21);
  if (res[2]!=0) res[2][12]=a1;
  a15=(a15*a12);
  a2=(a2*a15);
  a3=(a3*a17);
  a2=(a2+a3);
  a7=(a7*a18);
  a0=(a0*a12);
  a5=(a5*a0);
  a7=(a7-a5);
  a2=(a2+a7);
  if (res[2]!=0) res[2][13]=a2;
  a2=arg[2]? arg[2][17] : 0;
  if (res[2]!=0) res[2][14]=a2;
  a8=(a8*a17);
  if (res[2]!=0) res[2][15]=a8;
  a9=(a9*a18);
  if (res[2]!=0) res[2][16]=a9;
  a11=(a11*a2);
  a13=(a13+a11);
  if (res[2]!=0) res[2][17]=a13;
  return 0;
}

CASADI_SYMBOL_EXPORT int vessel_ode_expl_vde_forw(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int vessel_ode_expl_vde_forw_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int vessel_ode_expl_vde_forw_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void vessel_ode_expl_vde_forw_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int vessel_ode_expl_vde_forw_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void vessel_ode_expl_vde_forw_release(int mem) {
}

CASADI_SYMBOL_EXPORT void vessel_ode_expl_vde_forw_incref(void) {
}

CASADI_SYMBOL_EXPORT void vessel_ode_expl_vde_forw_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int vessel_ode_expl_vde_forw_n_in(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_int vessel_ode_expl_vde_forw_n_out(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_real vessel_ode_expl_vde_forw_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* vessel_ode_expl_vde_forw_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* vessel_ode_expl_vde_forw_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* vessel_ode_expl_vde_forw_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s3;
    case 4: return casadi_s4;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* vessel_ode_expl_vde_forw_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int vessel_ode_expl_vde_forw_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 5;
  if (sz_res) *sz_res = 3;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif

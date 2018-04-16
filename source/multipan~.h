#ifndef YC_MULTIPAN_H_
#define YC_MULTIPAN_H_

// From Mac

// ========  INCLUDES  ========

#include "max_util.h"
#include "envelopes.h"

// ========  DEFINES  ========

#define DEF_CHAN_COUNT  8
#define DEF_RAMP       10
#define INDEFINITE     -1

/* XXX #define CROSSING_NONE    0
#define CROSSING_UP      1
#define CROSSING_DOWN    2*/

// ========  STRUCTURE:  MULTIPAN  ========

typedef struct _multipan {

  t_pxobject obj;          // Use t_pxobject for MSP objects

  void*     outl_mess;     // Last outlet: for messages

  t_double  X_cur;
  t_int32   I_cur;
  t_double  R_cur;

  t_double  X_targ;
  t_int32   I_targ;
  t_double  R_targ;

  t_double  gain1;
  t_double  gain2;
  t_double  master;       // Master gain

  t_int32    channel_cnt; // Number of input channels

  t_double  ramp_ms;
  t_int32    ramp;
  t_int32    cntd;

  t_double  samplerate;   // Stores the samplerate
  t_double  msr;          // The samplerate in milliseconds

} t_multipan;

// ========  METHOD PROTOTYPES  ========

// ========  MAX MSP METHODS  ========

void* multipan_new       (t_symbol* sym, t_int32 argc, t_atom* argv);
void  multipan_free      (t_multipan* x);
void  multipan_dsp64     (t_multipan* x, t_object* dsp64, t_int32* count, t_double samplerate, long maxvectorsize, long flags);
void  multipan_perform64 (t_multipan* x, t_object* dsp64, t_double** ins, long numins, t_double** outs, long numouts, long sampleframes, long flags, void* userparam);
void  multipan_assist    (t_multipan* x, void* b, long msg, t_int32 arg, char* str);

// ======== MULTIPAN METHODS ========

void multipan_reset  (t_multipan* x);
void multipan_int    (t_multipan* x, long interp);
void multipan_float  (t_multipan* x, double interp);
void multipan_master (t_multipan* x, double master);
void multipan_ramp   (t_multipan* x, double ramp);
void multipan_post   (t_multipan* x);

#endif

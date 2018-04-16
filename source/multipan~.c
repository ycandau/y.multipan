//******************************************************************************
//  @file
//  multipan~ - Pan across a number of channels
//  Yves Candau - ycandau@gmail.com
//
//  @ingroup  myExternals
//

// ========  HEADER FILES  ========

#include "multipan~.h"

// ========  GLOBAL CLASS POINTER AND STATIC VARIABLES  ========

static t_class* multipan_class = NULL;

// ========  INITIALIZATION ROUTINE  ========

// ========  MAIN  ========

int C74_EXPORT main(void) {

  t_class* c = class_new("y.multipan~", (method)multipan_new, (method)multipan_free, (long)sizeof(t_multipan), 0L, A_GIMME, 0);

  // ====  MAX MSP METHODS  ====

  class_addmethod(c, (method)multipan_dsp64, "dsp64", A_CANT, 0);
  class_addmethod(c, (method)multipan_assist, "assist", A_CANT, 0);

  // ==== MULTIPAN METHODS ====

  class_addmethod(c, (method)multipan_reset, "reset", 0);
  class_addmethod(c, (method)multipan_int, "int", A_LONG, 0);
  class_addmethod(c, (method)multipan_float, "float", A_FLOAT, 0);
  class_addmethod(c, (method)multipan_master, "master", A_FLOAT, 0);
  class_addmethod(c, (method)multipan_ramp, "ramp", A_FLOAT, 0);
  class_addmethod(c, (method)multipan_post, "post", 0);

  class_dspinit(c);
  class_register(CLASS_BOX, c);
  multipan_class = c;

  return 0;
}

// ========  NEW INSTANCE ROUTINE: MULTIPAN_NEW  ========
//******************************************************************************
//  Called when the object is created.
//  Arguments:
//  (int: input channels) (int: output channels) [int: storage slots]
//
void* multipan_new(t_symbol* sym, t_int32 argc, t_atom* argv) {

  // ==== MAX initializations

  t_multipan* x = NULL;
  x = (t_multipan*)object_alloc(multipan_class);

  if (x == NULL) {
    MY_ERR("Object allocation failed.");
    return NULL;
  }

  TRACE("multipan_new");

  // ==== Arguments
  // [int: input channels] [float: ramp time in ms]

  // If two arguments are provided
  if ((argc == 2)
      && (atom_gettype(argv) == A_LONG) && (atom_getlong(argv) >= 2)
      && ((atom_gettype(argv + 1) == A_FLOAT) || (atom_gettype(argv + 1) == A_LONG))
      && (atom_getfloat(argv + 1) >= 1)) {

    x->channel_cnt = (t_int32)atom_getlong(argv);
    x->ramp_ms = (t_double)atom_getfloat(argv + 1);
  }

  // If one argument is provided
  else if ((argc == 1)
    && (atom_gettype(argv) == A_LONG) && (atom_getlong(argv) >= 2)) {

    x->channel_cnt = (t_int32)atom_getlong(argv);
    x->ramp_ms = DEF_RAMP;
  }

  // If zero arguments are provided, or otherwise
  else {
    x->channel_cnt = DEF_CHAN_COUNT;
    x->ramp_ms = DEF_RAMP;

    if (argc != 0) {
      MY_ERR("multipan:  Invalid arguments. The object expects:");
      MY_ERR2("  [int: input channels] [float: ramp time in ms]");
    }
  }

  // ==== Inlets and oulets

  // Create the MSP inlets
  dsp_setup((t_pxobject*)x, x->channel_cnt);

  // The second outlet is for messages
  x->outl_mess = outlet_new((t_object*)x, NULL);

  // The first outlet is for signals
  outlet_new((t_object*)x, "signal");

  // Separate the arrays for inlets and outlets
  // XXX x->obj.z_misc |= Z_NO_INPLACE;

  // ==== Initialization

  x->I_cur = 0;
  x->R_cur = 0;
  x->X_cur = 0;

  x->I_targ = 0;
  x->R_targ = 0;
  x->X_targ = 0;

  x->gain1 = 1.0;
  x->gain2 = 0.0;
  x->master = 1.0;
  x->samplerate = sys_getsr();
  x->msr = x->samplerate / 1000;
  x->ramp = (t_int32)(x->ramp_ms * x->msr);
  x->cntd = INDEFINITE;

  // Post a creation message
  POST("New object created:  %i input channels, %.2f ms ramping time",
    x->channel_cnt, x->ramp_ms);

  return (x);
}

// ========  METHOD: MULTIPAN_FREE  ========
// Called when the object is deleted

void multipan_free(t_multipan* x) {

  TRACE("multipan_free");

  dsp_free((t_pxobject*)x);
}

// ========  METHOD: MULTIPAN_DSP64  ========
// Called when the DAC is enabled

void multipan_dsp64(t_multipan* x, t_object* dsp64, t_int32* count, t_double samplerate, long maxvectorsize, long flags) {

  TRACE("multipan_dsp64");

  object_method(dsp64, gensym("dsp_add64"), x, multipan_perform64, 0, NULL);

  // Recalculate everything that depends on the samplerate
  x->samplerate = samplerate;
  x->msr = x->samplerate / 1000;
  x->ramp = (t_int32)(x->ramp_ms * x->msr);
}

// ========  METHOD: MULTIPAN_PERFORM64  ========

void multipan_perform64(t_multipan* x, t_object* dsp64, t_double** in_arr, long numins,
  t_double** out_arr, long numouts, long sampleframes, long flags, void* userparam) {

  // ####  LOOP THROUGH THE CHUNKS  ####

  // XXX Just for checking and catch errors that should not happen
  if ((x->cntd == 0)
    || ((x->X_targ == x->X_cur) && (x->cntd != INDEFINITE))) {
    MY_ERR("perform:  Countdown issue. Cntd = %i - X_cur = %.2f - I_cur = %i - R_cur = %.2f - X_targ = %.2f - I_targ = %i - R_targ = %.2f",
      x->cntd, x->X_cur, x->I_cur, x->R_cur, x->X_targ, x->I_targ, x->R_targ);
    x->cntd = INDEFINITE;
  }

  t_int32 chunk_len = 0;
  t_int32 smp_left = sampleframes;
  t_int32 smp_proc = 0;

  t_double dX = 0, R2 = 0, dG1 = 0, dG2 = 0;
  t_int32 I2 = 0;
  // t_int32 crossing = -1; @TEST

  t_int32 iter;
  t_double* out = out_arr[0];    // The output pointer will iterate continuously
  t_double* in1 = NULL;          // But the input pointers will jump between channels
  t_double* in2 = NULL;

  while (smp_left) {

    // ==== Determine the chunk length and update the countdown and smp_left accordingly
    // It depends on comparing the signal vector length with the countdown,
    // and also in this case on whether the interpolation value crosses a whole number

    // ==== If the countdown is indefinite...
    if (x->cntd == INDEFINITE) {

      // Process all the remaining samples without ramping
      chunk_len = smp_left;
      iter = chunk_len;

      // == For either one active channel
      if (x->R_cur == 0) {

        in1 = in_arr[x->I_cur] + smp_proc;
        // XXX out = out_arr[0] + smp_proc;
        while (iter--) { *out++ = *in1++; }
      }

      // == Or two active channels
      else {

        in1 = in_arr[x->I_cur] + smp_proc;
        in2 = in_arr[x->I_cur + 1] + smp_proc;
        // XXX out = out_arr[0] + smp_proc;
        while (iter--) { *out++ = x->gain1 * *(in1++) + x->gain2 * *(in2++); }
      }
    }

    // ==== Otherwise ramp the interpolation value and gains
    else {

      // Calculate the interpolation delta
      dX = (x->X_targ - x->X_cur) / x->cntd;    // XXX countdown cannot be 0

      // ==== COUNTDOWN AND SAMPLES LEFT
      // Calculate the chunk length depending on the countdown and length of samples left
      // and the floor of the interpolation value at the end
      if (x->cntd > smp_left) {
        chunk_len = smp_left;
        I2 = (t_int32)floor(x->X_cur + smp_left * dX);
      }

      else {
        chunk_len = x->cntd;
        I2 = x->I_targ;
      }

      // ==== CROSSINGS
      // Test whether the interpolation value crosses a whole number
      // in which case the chunk length has to be shortened
      if (I2 == x->I_cur) {; }
      else if (I2 > x->I_cur) { chunk_len = (t_int32)ceil((1 - x->R_cur) / dX); }    // Crossing UP
      else                    { chunk_len = (t_int32)floor(-x->R_cur / dX) + 1; }    // Crossing DOWN

      // XXX Check the rounding values
      if ((chunk_len > x->cntd) || (chunk_len > smp_left)) {
        POST("perform:  Rounding error corrected. Chunk: %i - Cntd = %i - Smp_left: %i - X_cur: %.2f - X_targ: %.2f",
          chunk_len, x->cntd, smp_left, x->X_cur, x->X_targ);
        chunk_len = MIN(x->cntd, smp_left);
      }

      // Calculate the gains at the end of the chunk length
      R2 = x->R_cur + chunk_len * dX;
      dG1 = (xfade_sinus(1 - R2, 0) - x->gain1) / chunk_len;
      dG2 = (xfade_sinus(R2, 0) - x->gain2) / chunk_len;

      // ==== SAMPLE LOOP
      // Loop through the samples
      iter = chunk_len;
      in1 = in_arr[x->I_cur] + smp_proc;
      in2 = in_arr[MIN(x->I_cur + 1, x->channel_cnt - 1)] + smp_proc;
      // XXX out = out_arr[0] + smp_proc;
      while (iter--) {
        *out++ = x->gain1 * *(in1++) + x->gain2 * *(in2++);
        x->gain1 += dG1;
        x->gain2 += dG2;
      }

    // Update the countdown
    x->cntd -= chunk_len;

    // Update the current interpolation values
    // and check for the end of a ramp
    if (x->cntd == 0) {
      x->X_cur = x->X_targ;
      x->I_cur = x->I_targ;
      x->R_cur = x->R_targ;
      x->cntd = INDEFINITE;
    }

    else /* XXX if (x->cntd > 0)*/ {
      x->X_cur += chunk_len * dX;
      x->I_cur = (t_int32)floor(x->X_cur);
      x->R_cur = x->X_cur - x->I_cur;
    }

    // XXX else { POST("perform:  negative countdown. %i", x->cntd); }  // XXX just in case

    // Update the gains
    x->gain1 = xfade_sinus(1 - x->R_cur, 0);
    x->gain2 = xfade_sinus(x->R_cur, 0);

    /* XXX if (crossing == CROSSING_UP) {
      POST("CROSSING UP: Cntd = %i - X_cur = %.2f - I_cur = %i - R_cur = %.2f - X_targ = %.2f - I_targ = %i - R_targ = %.2f",
        x->cntd, x->X_cur, x->I_cur, x->R_cur, x->X_targ, x->I_targ, x->R_targ);
      }

    else if (crossing == CROSSING_DOWN) {
      POST("CROSSING DOWN: Cntd = %i - X_cur = %.2f - I_cur = %i - R_cur = %.2f - X_targ = %.2f - I_targ = %i - R_targ = %.2f",
        x->cntd, x->X_cur, x->I_cur, x->R_cur, x->X_targ, x->I_targ, x->R_targ);
      } */

    }  // End of ramping case

    smp_left -= chunk_len;
    smp_proc += chunk_len;

  }  // End of the loop through the chunks
}

// ========  METHOD: MULTIPAN_ASSIST  ========

void multipan_assist(t_multipan* x, void* b, long msg, t_int32 arg, char* str) {

  if (msg == ASSIST_INLET) {

    switch (arg) {
    case 0: sprintf(str, "Inlet %i: All purpose and Input Channel %i (list / signal)", arg, arg); break;
    default: sprintf(str, "Inlet %i: Input Channel %i (signal)", arg, arg); break;
  }
}

  else if (msg == ASSIST_OUTLET) {

    switch (arg) {
    case 0: sprintf(str, "Outlet %i: Output Channel (signal)", arg); break;
    case 1: sprintf(str, "Outlet %i: All purpose messages (list)", arg); break;
  }
}
}

// ========  MULTIPAN METHODS  ========

// ====  MULTIPAN_RESET  ====

void multipan_reset(t_multipan* x) {

  ;
}

// ====  MULTIPAN_INT  ====

void multipan_int(t_multipan* x, long interp) {

  if (interp == x->X_targ) { return; }

  if (interp <= 0) {
    x->I_targ = 0;
    x->X_targ = 0;
  }

  else if (interp <= x->channel_cnt - 1) {
    x->I_targ = interp;
    x->X_targ = interp;
  }

  else {
    x->I_targ = x->channel_cnt - 1;
    x->X_targ = x->I_targ;
  }

  x->R_targ = 0;
  x->cntd = x->ramp;
}

// ====  MULTIPAN_FLOAT  ====

void multipan_float(t_multipan* x, double interp) {

  if (interp == x->X_targ) { return; }

  if (interp <= 0) {
    x->I_targ = 0;
    x->R_targ = 0;
    x->X_targ = 0;
  }

  else if (interp <= x->channel_cnt - 1) {
    x->I_targ = (t_int32)floor(interp);
    x->R_targ = interp - x->I_targ;
    x->X_targ = interp;
  }

  else {
    x->I_targ = x->channel_cnt - 1;
    x->R_targ = 0;
    x->X_targ = x->I_targ;
  }

  x->cntd = x->ramp;
}

// ====  MULTIPAN_MASTER  ====

void multipan_master(t_multipan* x, double master) {

  x->master = (t_double)master;
}

// ====  MULTIPAN_RAMP  ====

void multipan_ramp(t_multipan* x, double ramp) {

  x->ramp_ms = (t_double)ramp;
  x->ramp = (t_int32)(x->ramp_ms * x->msr);
}

// ====  MULTIPAN_POST  ====

void multipan_post(t_multipan* x) {

  POST("post:  Ch: %i - Ramp (ms): %.1f - Ramp (smp): %i - X_cur = %.2f - I_cur = %i - R_cur = %.2f",
    x->channel_cnt, x->ramp_ms, x->ramp, x->X_cur, x->I_cur, x->R_cur);
  POST("  X_targ = %.2f - I_targ = %i - R_targ = %.2f - Gain1 = %f - Gain2 = %f - Master = %f - Cntd: %i",
    x->X_targ, x->I_targ, x->R_targ, x->gain1, x->gain2, x->master, x->cntd);
}

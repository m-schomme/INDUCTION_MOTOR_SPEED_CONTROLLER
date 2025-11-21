#ifndef ACIM_H
#define ACIM_H

//
// ACIM Reference Generator
// (Computes id_ref and iq_ref for FOC)
//

typedef struct
{
    float P;          // pole pairs
    float Lm;         // magnetizing inductance
    float Lr;         // rotor inductance

    float psi_r_ref;  // desired rotor flux reference (Wb)
} ACIM_Params_t;


// Initializes ACIM parameters
void ACIM_Init(ACIM_Params_t *p);


// Computes id_ref and iq_ref given torque + speed references
// Normally omega_ref influences flux weakening (optional)
// For now omega_ref is included for future expansion
void ACIM_Compute(float omega_ref,
                  float torque_ref,
                  float *id_ref,
                  float *iq_ref);

#endif

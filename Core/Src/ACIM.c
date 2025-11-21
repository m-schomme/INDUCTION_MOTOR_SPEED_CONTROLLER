#include "ACIM.h"

static ACIM_Params_t prm;

void ACIM_Init(ACIM_Params_t *p)
{
    prm = *p;
}

void ACIM_Compute(float omega_ref,
                  float torque_ref,
                  float *id_ref,
                  float *iq_ref)
{
    //
    // 1) FLUX-PRODUCING CURRENT
    //    id_ref = psi_ref / Lm
    //
    *id_ref = prm.psi_r_ref / prm.Lm;


    //
    // 2) TORQUE-PRODUCING CURRENT
    //
    //      Te = (3/2) * P * (Lm^2/Lr) * id * iq
    //  =>  iq_ref = Tref / (Kt * id_ref)
    //
    float Kt = 1.5f * prm.P * (prm.Lm * prm.Lm / prm.Lr);

    if (*id_ref != 0.0f)
        *iq_ref = torque_ref / (Kt * (*id_ref));
    else
        *iq_ref = 0.0f;


    //
    // OPTIONAL (future): Flux Weakening
    // Use omega_ref to reduce psi_r_ref above base speed
    //
}

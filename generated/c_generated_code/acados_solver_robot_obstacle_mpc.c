/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */

// standard
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
// acados
// #include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

// example specific

#include "robot_obstacle_mpc_model/robot_obstacle_mpc_model.h"


#include "robot_obstacle_mpc_constraints/robot_obstacle_mpc_constraints.h"



#include "acados_solver_robot_obstacle_mpc.h"

#define NX     ROBOT_OBSTACLE_MPC_NX
#define NZ     ROBOT_OBSTACLE_MPC_NZ
#define NU     ROBOT_OBSTACLE_MPC_NU
#define NP     ROBOT_OBSTACLE_MPC_NP
#define NP_GLOBAL     ROBOT_OBSTACLE_MPC_NP_GLOBAL
#define NY0    ROBOT_OBSTACLE_MPC_NY0
#define NY     ROBOT_OBSTACLE_MPC_NY
#define NYN    ROBOT_OBSTACLE_MPC_NYN

#define NBX    ROBOT_OBSTACLE_MPC_NBX
#define NBX0   ROBOT_OBSTACLE_MPC_NBX0
#define NBU    ROBOT_OBSTACLE_MPC_NBU
#define NG     ROBOT_OBSTACLE_MPC_NG
#define NBXN   ROBOT_OBSTACLE_MPC_NBXN
#define NGN    ROBOT_OBSTACLE_MPC_NGN

#define NH     ROBOT_OBSTACLE_MPC_NH
#define NHN    ROBOT_OBSTACLE_MPC_NHN
#define NH0    ROBOT_OBSTACLE_MPC_NH0
#define NPHI   ROBOT_OBSTACLE_MPC_NPHI
#define NPHIN  ROBOT_OBSTACLE_MPC_NPHIN
#define NPHI0  ROBOT_OBSTACLE_MPC_NPHI0
#define NR     ROBOT_OBSTACLE_MPC_NR

#define NS     ROBOT_OBSTACLE_MPC_NS
#define NS0    ROBOT_OBSTACLE_MPC_NS0
#define NSN    ROBOT_OBSTACLE_MPC_NSN

#define NSBX   ROBOT_OBSTACLE_MPC_NSBX
#define NSBU   ROBOT_OBSTACLE_MPC_NSBU
#define NSH0   ROBOT_OBSTACLE_MPC_NSH0
#define NSH    ROBOT_OBSTACLE_MPC_NSH
#define NSHN   ROBOT_OBSTACLE_MPC_NSHN
#define NSG    ROBOT_OBSTACLE_MPC_NSG
#define NSPHI0 ROBOT_OBSTACLE_MPC_NSPHI0
#define NSPHI  ROBOT_OBSTACLE_MPC_NSPHI
#define NSPHIN ROBOT_OBSTACLE_MPC_NSPHIN
#define NSGN   ROBOT_OBSTACLE_MPC_NSGN
#define NSBXN  ROBOT_OBSTACLE_MPC_NSBXN



// ** solver data **

robot_obstacle_mpc_solver_capsule * robot_obstacle_mpc_acados_create_capsule(void)
{
    void* capsule_mem = malloc(sizeof(robot_obstacle_mpc_solver_capsule));
    robot_obstacle_mpc_solver_capsule *capsule = (robot_obstacle_mpc_solver_capsule *) capsule_mem;

    return capsule;
}


int robot_obstacle_mpc_acados_free_capsule(robot_obstacle_mpc_solver_capsule *capsule)
{
    free(capsule);
    return 0;
}


int robot_obstacle_mpc_acados_create(robot_obstacle_mpc_solver_capsule* capsule)
{
    int N_shooting_intervals = ROBOT_OBSTACLE_MPC_N;
    double* new_time_steps = NULL; // NULL -> don't alter the code generated time-steps
    return robot_obstacle_mpc_acados_create_with_discretization(capsule, N_shooting_intervals, new_time_steps);
}


int robot_obstacle_mpc_acados_update_time_steps(robot_obstacle_mpc_solver_capsule* capsule, int N, double* new_time_steps)
{

    if (N != capsule->nlp_solver_plan->N) {
        fprintf(stderr, "robot_obstacle_mpc_acados_update_time_steps: given number of time steps (= %d) " \
            "differs from the currently allocated number of " \
            "time steps (= %d)!\n" \
            "Please recreate with new discretization and provide a new vector of time_stamps!\n",
            N, capsule->nlp_solver_plan->N);
        return 1;
    }

    ocp_nlp_config * nlp_config = capsule->nlp_config;
    ocp_nlp_dims * nlp_dims = capsule->nlp_dims;
    ocp_nlp_in * nlp_in = capsule->nlp_in;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &new_time_steps[i]);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &new_time_steps[i]);
    }
    return 0;

}

/**
 * Internal function for robot_obstacle_mpc_acados_create: step 1
 */
void robot_obstacle_mpc_acados_create_set_plan(ocp_nlp_plan_t* nlp_solver_plan, const int N)
{
    assert(N == nlp_solver_plan->N);

    /************************************************
    *  plan
    ************************************************/

    nlp_solver_plan->nlp_solver = SQP;

    nlp_solver_plan->ocp_qp_solver_plan.qp_solver = PARTIAL_CONDENSING_HPIPM;
    nlp_solver_plan->relaxed_ocp_qp_solver_plan.qp_solver = PARTIAL_CONDENSING_HPIPM;
    nlp_solver_plan->nlp_cost[0] = LINEAR_LS;
    for (int i = 1; i < N; i++)
        nlp_solver_plan->nlp_cost[i] = LINEAR_LS;

    nlp_solver_plan->nlp_cost[N] = LINEAR_LS;

    for (int i = 0; i < N; i++)
    {
        nlp_solver_plan->nlp_dynamics[i] = CONTINUOUS_MODEL;
        nlp_solver_plan->sim_solver_plan[i].sim_solver = ERK;
    }

    nlp_solver_plan->nlp_constraints[0] = BGH;

    for (int i = 1; i < N; i++)
    {
        nlp_solver_plan->nlp_constraints[i] = BGH;
    }
    nlp_solver_plan->nlp_constraints[N] = BGH;

    nlp_solver_plan->regularization = PROJECT;

    nlp_solver_plan->globalization = FIXED_STEP;
}


static ocp_nlp_dims* robot_obstacle_mpc_acados_create_setup_dimensions(robot_obstacle_mpc_solver_capsule* capsule)
{
    ocp_nlp_plan_t* nlp_solver_plan = capsule->nlp_solver_plan;
    const int N = nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;

    /************************************************
    *  dimensions
    ************************************************/
    #define NINTNP1MEMS 18
    int* intNp1mem = (int*)malloc( (N+1)*sizeof(int)*NINTNP1MEMS );

    int* nx    = intNp1mem + (N+1)*0;
    int* nu    = intNp1mem + (N+1)*1;
    int* nbx   = intNp1mem + (N+1)*2;
    int* nbu   = intNp1mem + (N+1)*3;
    int* nsbx  = intNp1mem + (N+1)*4;
    int* nsbu  = intNp1mem + (N+1)*5;
    int* nsg   = intNp1mem + (N+1)*6;
    int* nsh   = intNp1mem + (N+1)*7;
    int* nsphi = intNp1mem + (N+1)*8;
    int* ns    = intNp1mem + (N+1)*9;
    int* ng    = intNp1mem + (N+1)*10;
    int* nh    = intNp1mem + (N+1)*11;
    int* nphi  = intNp1mem + (N+1)*12;
    int* nz    = intNp1mem + (N+1)*13;
    int* ny    = intNp1mem + (N+1)*14;
    int* nr    = intNp1mem + (N+1)*15;
    int* nbxe  = intNp1mem + (N+1)*16;
    int* np  = intNp1mem + (N+1)*17;

    for (int i = 0; i < N+1; i++)
    {
        // common
        nx[i]     = NX;
        nu[i]     = NU;
        nz[i]     = NZ;
        ns[i]     = NS;
        // cost
        ny[i]     = NY;
        // constraints
        nbx[i]    = NBX;
        nbu[i]    = NBU;
        nsbx[i]   = NSBX;
        nsbu[i]   = NSBU;
        nsg[i]    = NSG;
        nsh[i]    = NSH;
        nsphi[i]  = NSPHI;
        ng[i]     = NG;
        nh[i]     = NH;
        nphi[i]   = NPHI;
        nr[i]     = NR;
        nbxe[i]   = 0;
        np[i]     = NP;
    }

    // for initial state
    nbx[0] = NBX0;
    nsbx[0] = 0;
    ns[0] = NS0;
    
    nbxe[0] = 6;
    
    ny[0] = NY0;
    nh[0] = NH0;
    nsh[0] = NSH0;
    nsphi[0] = NSPHI0;
    nphi[0] = NPHI0;


    // terminal - common
    nu[N]   = 0;
    nz[N]   = 0;
    ns[N]   = NSN;
    // cost
    ny[N]   = NYN;
    // constraint
    nbx[N]   = NBXN;
    nbu[N]   = 0;
    ng[N]    = NGN;
    nh[N]    = NHN;
    nphi[N]  = NPHIN;
    nr[N]    = 0;

    nsbx[N]  = NSBXN;
    nsbu[N]  = 0;
    nsg[N]   = NSGN;
    nsh[N]   = NSHN;
    nsphi[N] = NSPHIN;

    /* create and set ocp_nlp_dims */
    ocp_nlp_dims * nlp_dims = ocp_nlp_dims_create(nlp_config);

    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nx", nx);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nu", nu);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nz", nz);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "ns", ns);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "np", np);

    ocp_nlp_dims_set_global(nlp_config, nlp_dims, "np_global", 0);
    ocp_nlp_dims_set_global(nlp_config, nlp_dims, "n_global_data", 0);

    for (int i = 0; i <= N; i++)
    {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbx", &nbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbu", &nbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbx", &nsbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbu", &nsbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "ng", &ng[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsg", &nsg[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbxe", &nbxe[i]);
    }
    ocp_nlp_dims_set_cost(nlp_config, nlp_dims, 0, "ny", &ny[0]);
    for (int i = 1; i < N; i++)
        ocp_nlp_dims_set_cost(nlp_config, nlp_dims, i, "ny", &ny[i]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, 0, "nh", &nh[0]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, 0, "nsh", &nsh[0]);

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nh", &nh[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsh", &nsh[i]);
    }
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nh", &nh[N]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nsh", &nsh[N]);
    ocp_nlp_dims_set_cost(nlp_config, nlp_dims, N, "ny", &ny[N]);
    free(intNp1mem);

    return nlp_dims;
}


/**
 * Internal function for robot_obstacle_mpc_acados_create: step 3
 */
void robot_obstacle_mpc_acados_create_setup_functions(robot_obstacle_mpc_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;

    /************************************************
    *  external functions
    ************************************************/

#define MAP_CASADI_FNC(__CAPSULE_FNC__, __MODEL_BASE_FNC__) do{ \
        capsule->__CAPSULE_FNC__.casadi_fun = & __MODEL_BASE_FNC__ ;\
        capsule->__CAPSULE_FNC__.casadi_n_in = & __MODEL_BASE_FNC__ ## _n_in; \
        capsule->__CAPSULE_FNC__.casadi_n_out = & __MODEL_BASE_FNC__ ## _n_out; \
        capsule->__CAPSULE_FNC__.casadi_sparsity_in = & __MODEL_BASE_FNC__ ## _sparsity_in; \
        capsule->__CAPSULE_FNC__.casadi_sparsity_out = & __MODEL_BASE_FNC__ ## _sparsity_out; \
        capsule->__CAPSULE_FNC__.casadi_work = & __MODEL_BASE_FNC__ ## _work; \
        external_function_external_param_casadi_create(&capsule->__CAPSULE_FNC__, &ext_fun_opts); \
    } while(false)

    external_function_opts ext_fun_opts;
    external_function_opts_set_to_default(&ext_fun_opts);


    ext_fun_opts.external_workspace = true;
    if (N > 0)
    {
        // constraints.constr_type == "BGH" and dims.nh > 0
        capsule->nl_constr_h_fun_jac = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
        for (int i = 0; i < N-1; i++) {
            MAP_CASADI_FNC(nl_constr_h_fun_jac[i], robot_obstacle_mpc_constr_h_fun_jac_uxt_zt);
        }
        capsule->nl_constr_h_fun = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
        for (int i = 0; i < N-1; i++) {
            MAP_CASADI_FNC(nl_constr_h_fun[i], robot_obstacle_mpc_constr_h_fun);
        }
    



    
        // explicit ode
        capsule->expl_vde_forw = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*N);
        for (int i = 0; i < N; i++) {
            MAP_CASADI_FNC(expl_vde_forw[i], robot_obstacle_mpc_expl_vde_forw);
        }

        

        capsule->expl_ode_fun = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*N);
        for (int i = 0; i < N; i++) {
            MAP_CASADI_FNC(expl_ode_fun[i], robot_obstacle_mpc_expl_ode_fun);
        }

        capsule->expl_vde_adj = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*N);
        for (int i = 0; i < N; i++) {
            MAP_CASADI_FNC(expl_vde_adj[i], robot_obstacle_mpc_expl_vde_adj);
        }

    
    } // N > 0

#undef MAP_CASADI_FNC
}


/**
 * Internal function for robot_obstacle_mpc_acados_create: step 5
 */
void robot_obstacle_mpc_acados_create_set_default_parameters(robot_obstacle_mpc_solver_capsule* capsule)
{

    const int N = capsule->nlp_solver_plan->N;
    // initialize parameters to nominal value
    double* p = calloc(NP, sizeof(double));
    p[0] = 1000000;
    p[1] = 1000000;
    p[2] = 1000000;
    p[3] = 1000000;
    p[4] = 1000000;
    p[5] = 1000000;
    p[6] = 1000000;
    p[7] = 1000000;
    p[8] = 1000000;
    p[9] = 1000000;
    p[10] = 1000000;
    p[11] = 1000000;
    p[12] = 1000000;
    p[13] = 1000000;
    p[14] = 1000000;
    p[15] = 1000000;
    p[16] = 1000000;
    p[17] = 1000000;
    p[18] = 1000000;
    p[19] = 1000000;
    p[20] = 1000000;
    p[21] = 1000000;
    p[22] = 1000000;
    p[23] = 1000000;
    p[24] = 1000000;
    p[25] = 1000000;
    p[26] = 1000000;
    p[27] = 1000000;
    p[28] = 1000000;
    p[29] = 1000000;

    for (int i = 0; i <= N; i++) {
        robot_obstacle_mpc_acados_update_params(capsule, i, p, NP);
    }
    free(p);


    // no global parameters defined
}


/**
 * Internal function for robot_obstacle_mpc_acados_create: step 5
 */
void robot_obstacle_mpc_acados_setup_nlp_in(robot_obstacle_mpc_solver_capsule* capsule, const int N, double* new_time_steps)
{
    assert(N == capsule->nlp_solver_plan->N);
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;

    int tmp_int = 0;

    /************************************************
    *  nlp_in
    ************************************************/
    ocp_nlp_in * nlp_in = capsule->nlp_in;
    /************************************************
    *  nlp_out
    ************************************************/
    ocp_nlp_out * nlp_out = capsule->nlp_out;

    // set up time_steps and cost_scaling

    if (new_time_steps)
    {
        // NOTE: this sets scaling and time_steps
        robot_obstacle_mpc_acados_update_time_steps(capsule, N, new_time_steps);
    }
    else
    {
        // set time_steps
    
        double time_step = 0.15;
        for (int i = 0; i < N; i++)
        {
            ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &time_step);
        }
        // set cost scaling
        double* cost_scaling = malloc((N+1)*sizeof(double));
        cost_scaling[0] = 0.15;
        cost_scaling[1] = 0.15;
        cost_scaling[2] = 0.15;
        cost_scaling[3] = 0.15;
        cost_scaling[4] = 0.15;
        cost_scaling[5] = 0.15;
        cost_scaling[6] = 0.15;
        cost_scaling[7] = 0.15;
        cost_scaling[8] = 0.15;
        cost_scaling[9] = 0.15;
        cost_scaling[10] = 0.15;
        cost_scaling[11] = 0.15;
        cost_scaling[12] = 0.15;
        cost_scaling[13] = 0.15;
        cost_scaling[14] = 0.15;
        cost_scaling[15] = 0.15;
        cost_scaling[16] = 0.15;
        cost_scaling[17] = 0.15;
        cost_scaling[18] = 0.15;
        cost_scaling[19] = 0.15;
        cost_scaling[20] = 1;
        for (int i = 0; i <= N; i++)
        {
            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &cost_scaling[i]);
        }
        free(cost_scaling);
    }



    /**** Dynamics ****/
    for (int i = 0; i < N; i++)
    {
        ocp_nlp_dynamics_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "expl_vde_forw", &capsule->expl_vde_forw[i]);
        
        ocp_nlp_dynamics_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "expl_ode_fun", &capsule->expl_ode_fun[i]);
        ocp_nlp_dynamics_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "expl_vde_adj", &capsule->expl_vde_adj[i]);
    }

    /**** Cost ****/
    double* yref_0 = calloc(NY0, sizeof(double));
    // change only the non-zero elements:
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "yref", yref_0);
    free(yref_0);

   double* W_0 = calloc(NY0*NY0, sizeof(double));
    // change only the non-zero elements:
    W_0[0+(NY0) * 0] = 15;
    W_0[1+(NY0) * 1] = 15;
    W_0[2+(NY0) * 2] = 2;
    W_0[3+(NY0) * 3] = 0.5;
    W_0[4+(NY0) * 4] = 0.5;
    W_0[5+(NY0) * 5] = 0.5;
    W_0[6+(NY0) * 6] = 0.1;
    W_0[7+(NY0) * 7] = 0.1;
    W_0[8+(NY0) * 8] = 0.1;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "W", W_0);
    free(W_0);
    double* Vx_0 = calloc(NY0*NX, sizeof(double));
    // change only the non-zero elements:
    Vx_0[0+(NY0) * 0] = 1;
    Vx_0[1+(NY0) * 1] = 1;
    Vx_0[2+(NY0) * 2] = 1;
    Vx_0[3+(NY0) * 3] = 1;
    Vx_0[4+(NY0) * 4] = 1;
    Vx_0[5+(NY0) * 5] = 1;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "Vx", Vx_0);
    free(Vx_0);
    double* Vu_0 = calloc(NY0*NU, sizeof(double));
    // change only the non-zero elements:
    Vu_0[6+(NY0) * 0] = 1;
    Vu_0[7+(NY0) * 1] = 1;
    Vu_0[8+(NY0) * 2] = 1;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "Vu", Vu_0);
    free(Vu_0);
    double* yref = calloc(NY, sizeof(double));
    // change only the non-zero elements:

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref);
    }
    free(yref);
    double* W = calloc(NY*NY, sizeof(double));
    // change only the non-zero elements:
    W[0+(NY) * 0] = 15;
    W[1+(NY) * 1] = 15;
    W[2+(NY) * 2] = 2;
    W[3+(NY) * 3] = 0.5;
    W[4+(NY) * 4] = 0.5;
    W[5+(NY) * 5] = 0.5;
    W[6+(NY) * 6] = 0.1;
    W[7+(NY) * 7] = 0.1;
    W[8+(NY) * 8] = 0.1;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "W", W);
    }
    free(W);
    double* Vx = calloc(NY*NX, sizeof(double));
    // change only the non-zero elements:
    Vx[0+(NY) * 0] = 1;
    Vx[1+(NY) * 1] = 1;
    Vx[2+(NY) * 2] = 1;
    Vx[3+(NY) * 3] = 1;
    Vx[4+(NY) * 4] = 1;
    Vx[5+(NY) * 5] = 1;
    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Vx", Vx);
    }
    free(Vx);

    
    double* Vu = calloc(NY*NU, sizeof(double));
    // change only the non-zero elements:
    Vu[6+(NY) * 0] = 1;
    Vu[7+(NY) * 1] = 1;
    Vu[8+(NY) * 2] = 1;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Vu", Vu);
    }
    free(Vu);
    double* yref_e = calloc(NYN, sizeof(double));
    // change only the non-zero elements:
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", yref_e);
    free(yref_e);

    double* W_e = calloc(NYN*NYN, sizeof(double));
    // change only the non-zero elements:
    W_e[0+(NYN) * 0] = 100;
    W_e[1+(NYN) * 1] = 100;
    W_e[2+(NYN) * 2] = 10;
    W_e[3+(NYN) * 3] = 5;
    W_e[4+(NYN) * 4] = 5;
    W_e[5+(NYN) * 5] = 2;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "W", W_e);
    free(W_e);
    double* Vx_e = calloc(NYN*NX, sizeof(double));
    // change only the non-zero elements:
    Vx_e[0+(NYN) * 0] = 1;
    Vx_e[1+(NYN) * 1] = 1;
    Vx_e[2+(NYN) * 2] = 1;
    Vx_e[3+(NYN) * 3] = 1;
    Vx_e[4+(NYN) * 4] = 1;
    Vx_e[5+(NYN) * 5] = 1;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "Vx", Vx_e);
    free(Vx_e);




    // slacks
    double* zlumem = calloc(4*NS, sizeof(double));
    double* Zl = zlumem+NS*0;
    double* Zu = zlumem+NS*1;
    double* zl = zlumem+NS*2;
    double* zu = zlumem+NS*3;
    // change only the non-zero elements:
    Zl[0] = 10;
    Zl[1] = 10;
    Zl[2] = 10;
    Zl[3] = 10;
    Zl[4] = 10;
    Zl[5] = 10;
    Zl[6] = 10;
    Zl[7] = 10;
    Zl[8] = 10;
    Zl[9] = 10;
    Zl[10] = 10;
    Zl[11] = 10;
    Zl[12] = 10;
    Zl[13] = 10;
    Zl[14] = 10;
    Zl[15] = 10;
    Zl[16] = 10;
    Zl[17] = 10;
    Zl[18] = 10;
    Zl[19] = 10;
    Zl[20] = 10;
    Zl[21] = 10;
    Zl[22] = 10;
    Zl[23] = 10;
    Zl[24] = 10;
    Zl[25] = 10;
    Zl[26] = 10;
    Zl[27] = 10;
    Zl[28] = 10;
    Zl[29] = 10;
    Zl[30] = 10;
    Zl[31] = 10;
    Zl[32] = 10;
    Zl[33] = 10;
    Zl[34] = 10;
    Zl[35] = 10;
    Zl[36] = 10;
    Zl[37] = 10;
    Zl[38] = 10;
    Zl[39] = 10;
    Zl[40] = 10;
    Zl[41] = 10;
    Zl[42] = 10;
    Zl[43] = 10;
    Zl[44] = 10;
    Zl[45] = 10;
    Zl[46] = 10;
    Zl[47] = 10;
    Zl[48] = 10;
    Zl[49] = 10;
    Zl[50] = 10;
    Zl[51] = 10;
    Zl[52] = 10;
    Zl[53] = 10;
    Zl[54] = 10;
    Zl[55] = 10;
    Zl[56] = 10;
    Zl[57] = 10;
    Zl[58] = 10;
    Zl[59] = 10;
    Zl[60] = 10;
    Zl[61] = 10;
    Zl[62] = 10;
    Zl[63] = 10;
    Zl[64] = 10;
    Zl[65] = 10;
    Zl[66] = 10;
    Zl[67] = 10;
    Zl[68] = 10;
    Zl[69] = 10;
    Zl[70] = 10;
    Zl[71] = 10;
    Zl[72] = 10;
    Zl[73] = 10;
    Zl[74] = 10;
    Zl[75] = 10;
    Zl[76] = 10;
    Zl[77] = 10;
    Zl[78] = 10;
    Zl[79] = 10;
    Zu[0] = 10;
    Zu[1] = 10;
    Zu[2] = 10;
    Zu[3] = 10;
    Zu[4] = 10;
    Zu[5] = 10;
    Zu[6] = 10;
    Zu[7] = 10;
    Zu[8] = 10;
    Zu[9] = 10;
    Zu[10] = 10;
    Zu[11] = 10;
    Zu[12] = 10;
    Zu[13] = 10;
    Zu[14] = 10;
    Zu[15] = 10;
    Zu[16] = 10;
    Zu[17] = 10;
    Zu[18] = 10;
    Zu[19] = 10;
    Zu[20] = 10;
    Zu[21] = 10;
    Zu[22] = 10;
    Zu[23] = 10;
    Zu[24] = 10;
    Zu[25] = 10;
    Zu[26] = 10;
    Zu[27] = 10;
    Zu[28] = 10;
    Zu[29] = 10;
    Zu[30] = 10;
    Zu[31] = 10;
    Zu[32] = 10;
    Zu[33] = 10;
    Zu[34] = 10;
    Zu[35] = 10;
    Zu[36] = 10;
    Zu[37] = 10;
    Zu[38] = 10;
    Zu[39] = 10;
    Zu[40] = 10;
    Zu[41] = 10;
    Zu[42] = 10;
    Zu[43] = 10;
    Zu[44] = 10;
    Zu[45] = 10;
    Zu[46] = 10;
    Zu[47] = 10;
    Zu[48] = 10;
    Zu[49] = 10;
    Zu[50] = 10;
    Zu[51] = 10;
    Zu[52] = 10;
    Zu[53] = 10;
    Zu[54] = 10;
    Zu[55] = 10;
    Zu[56] = 10;
    Zu[57] = 10;
    Zu[58] = 10;
    Zu[59] = 10;
    Zu[60] = 10;
    Zu[61] = 10;
    Zu[62] = 10;
    Zu[63] = 10;
    Zu[64] = 10;
    Zu[65] = 10;
    Zu[66] = 10;
    Zu[67] = 10;
    Zu[68] = 10;
    Zu[69] = 10;
    Zu[70] = 10;
    Zu[71] = 10;
    Zu[72] = 10;
    Zu[73] = 10;
    Zu[74] = 10;
    Zu[75] = 10;
    Zu[76] = 10;
    Zu[77] = 10;
    Zu[78] = 10;
    Zu[79] = 10;
    zl[0] = 10;
    zl[1] = 10;
    zl[2] = 10;
    zl[3] = 10;
    zl[4] = 10;
    zl[5] = 10;
    zl[6] = 10;
    zl[7] = 10;
    zl[8] = 10;
    zl[9] = 10;
    zl[10] = 10;
    zl[11] = 10;
    zl[12] = 10;
    zl[13] = 10;
    zl[14] = 10;
    zl[15] = 10;
    zl[16] = 10;
    zl[17] = 10;
    zl[18] = 10;
    zl[19] = 10;
    zl[20] = 10;
    zl[21] = 10;
    zl[22] = 10;
    zl[23] = 10;
    zl[24] = 10;
    zl[25] = 10;
    zl[26] = 10;
    zl[27] = 10;
    zl[28] = 10;
    zl[29] = 10;
    zl[30] = 10;
    zl[31] = 10;
    zl[32] = 10;
    zl[33] = 10;
    zl[34] = 10;
    zl[35] = 10;
    zl[36] = 10;
    zl[37] = 10;
    zl[38] = 10;
    zl[39] = 10;
    zl[40] = 10;
    zl[41] = 10;
    zl[42] = 10;
    zl[43] = 10;
    zl[44] = 10;
    zl[45] = 10;
    zl[46] = 10;
    zl[47] = 10;
    zl[48] = 10;
    zl[49] = 10;
    zl[50] = 10;
    zl[51] = 10;
    zl[52] = 10;
    zl[53] = 10;
    zl[54] = 10;
    zl[55] = 10;
    zl[56] = 10;
    zl[57] = 10;
    zl[58] = 10;
    zl[59] = 10;
    zl[60] = 10;
    zl[61] = 10;
    zl[62] = 10;
    zl[63] = 10;
    zl[64] = 10;
    zl[65] = 10;
    zl[66] = 10;
    zl[67] = 10;
    zl[68] = 10;
    zl[69] = 10;
    zl[70] = 10;
    zl[71] = 10;
    zl[72] = 10;
    zl[73] = 10;
    zl[74] = 10;
    zl[75] = 10;
    zl[76] = 10;
    zl[77] = 10;
    zl[78] = 10;
    zl[79] = 10;
    zu[0] = 10;
    zu[1] = 10;
    zu[2] = 10;
    zu[3] = 10;
    zu[4] = 10;
    zu[5] = 10;
    zu[6] = 10;
    zu[7] = 10;
    zu[8] = 10;
    zu[9] = 10;
    zu[10] = 10;
    zu[11] = 10;
    zu[12] = 10;
    zu[13] = 10;
    zu[14] = 10;
    zu[15] = 10;
    zu[16] = 10;
    zu[17] = 10;
    zu[18] = 10;
    zu[19] = 10;
    zu[20] = 10;
    zu[21] = 10;
    zu[22] = 10;
    zu[23] = 10;
    zu[24] = 10;
    zu[25] = 10;
    zu[26] = 10;
    zu[27] = 10;
    zu[28] = 10;
    zu[29] = 10;
    zu[30] = 10;
    zu[31] = 10;
    zu[32] = 10;
    zu[33] = 10;
    zu[34] = 10;
    zu[35] = 10;
    zu[36] = 10;
    zu[37] = 10;
    zu[38] = 10;
    zu[39] = 10;
    zu[40] = 10;
    zu[41] = 10;
    zu[42] = 10;
    zu[43] = 10;
    zu[44] = 10;
    zu[45] = 10;
    zu[46] = 10;
    zu[47] = 10;
    zu[48] = 10;
    zu[49] = 10;
    zu[50] = 10;
    zu[51] = 10;
    zu[52] = 10;
    zu[53] = 10;
    zu[54] = 10;
    zu[55] = 10;
    zu[56] = 10;
    zu[57] = 10;
    zu[58] = 10;
    zu[59] = 10;
    zu[60] = 10;
    zu[61] = 10;
    zu[62] = 10;
    zu[63] = 10;
    zu[64] = 10;
    zu[65] = 10;
    zu[66] = 10;
    zu[67] = 10;
    zu[68] = 10;
    zu[69] = 10;
    zu[70] = 10;
    zu[71] = 10;
    zu[72] = 10;
    zu[73] = 10;
    zu[74] = 10;
    zu[75] = 10;
    zu[76] = 10;
    zu[77] = 10;
    zu[78] = 10;
    zu[79] = 10;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Zl", Zl);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Zu", Zu);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "zl", zl);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "zu", zu);
    }
    free(zlumem);



    /**** Constraints ****/

    // bounds for initial stage
    // x0
    int* idxbx0 = malloc(NBX0 * sizeof(int));
    idxbx0[0] = 0;
    idxbx0[1] = 1;
    idxbx0[2] = 2;
    idxbx0[3] = 3;
    idxbx0[4] = 4;
    idxbx0[5] = 5;

    double* lubx0 = calloc(2*NBX0, sizeof(double));
    double* lbx0 = lubx0;
    double* ubx0 = lubx0 + NBX0;
    // change only the non-zero elements:

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", ubx0);
    free(idxbx0);
    free(lubx0);
    // idxbxe_0
    int* idxbxe_0 = malloc(6 * sizeof(int));
    idxbxe_0[0] = 0;
    idxbxe_0[1] = 1;
    idxbxe_0[2] = 2;
    idxbxe_0[3] = 3;
    idxbxe_0[4] = 4;
    idxbxe_0[5] = 5;
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "idxbxe", idxbxe_0);
    free(idxbxe_0);












    /* constraints that are the same for initial and intermediate */
    // u
    int* idxbu = malloc(NBU * sizeof(int));
    idxbu[0] = 0;
    idxbu[1] = 1;
    idxbu[2] = 2;
    double* lubu = calloc(2*NBU, sizeof(double));
    double* lbu = lubu;
    double* ubu = lubu + NBU;
    lbu[0] = -4;
    ubu[0] = 4;
    lbu[1] = -3;
    ubu[1] = 3;
    lbu[2] = -2;
    ubu[2] = 2;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "idxbu", idxbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "lbu", lbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "ubu", ubu);
    }
    free(idxbu);
    free(lubu);






    /* Path constraints */

    // x
    int* idxbx = malloc(NBX * sizeof(int));
    idxbx[0] = 3;
    idxbx[1] = 4;
    idxbx[2] = 5;
    double* lubx = calloc(2*NBX, sizeof(double));
    double* lbx = lubx;
    double* ubx = lubx + NBX;
    lbx[0] = -0.8;
    ubx[0] = 0.8;
    lbx[1] = -0.3;
    ubx[1] = 0.3;
    lbx[2] = -1.2;
    ubx[2] = 1.2;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "idxbx", idxbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "lbx", lbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "ubx", ubx);
    }
    free(idxbx);
    free(lubx);


    // set up nonlinear constraints for stage 1 to N-1
    double* luh = calloc(2*NH, sizeof(double));
    double* lh = luh;
    double* uh = luh + NH;
    uh[0] = 1000000;
    uh[1] = 1000000;
    uh[2] = 1000000;
    uh[3] = 1000000;
    uh[4] = 1000000;
    uh[5] = 1000000;
    uh[6] = 1000000;
    uh[7] = 1000000;
    uh[8] = 1000000;
    uh[9] = 1000000;
    uh[10] = 1000000;
    uh[11] = 1000000;
    uh[12] = 1000000;
    uh[13] = 1000000;
    uh[14] = 1000000;
    uh[15] = 1000000;
    uh[16] = 1000000;
    uh[17] = 1000000;
    uh[18] = 1000000;
    uh[19] = 1000000;
    uh[20] = 1000000;
    uh[21] = 1000000;
    uh[22] = 1000000;
    uh[23] = 1000000;
    uh[24] = 1000000;
    uh[25] = 1000000;
    uh[26] = 1000000;
    uh[27] = 1000000;
    uh[28] = 1000000;
    uh[29] = 1000000;
    uh[30] = 1000000;
    uh[31] = 1000000;
    uh[32] = 1000000;
    uh[33] = 1000000;
    uh[34] = 1000000;
    uh[35] = 1000000;
    uh[36] = 1000000;
    uh[37] = 1000000;
    uh[38] = 1000000;
    uh[39] = 1000000;
    uh[40] = 1000000;
    uh[41] = 1000000;
    uh[42] = 1000000;
    uh[43] = 1000000;
    uh[44] = 1000000;
    uh[45] = 1000000;
    uh[46] = 1000000;
    uh[47] = 1000000;
    uh[48] = 1000000;
    uh[49] = 1000000;
    uh[50] = 1000000;
    uh[51] = 1000000;
    uh[52] = 1000000;
    uh[53] = 1000000;
    uh[54] = 1000000;
    uh[55] = 1000000;
    uh[56] = 1000000;
    uh[57] = 1000000;
    uh[58] = 1000000;
    uh[59] = 1000000;
    uh[60] = 1000000;
    uh[61] = 1000000;
    uh[62] = 1000000;
    uh[63] = 1000000;
    uh[64] = 1000000;
    uh[65] = 1000000;
    uh[66] = 1000000;
    uh[67] = 1000000;
    uh[68] = 1000000;
    uh[69] = 1000000;
    uh[70] = 1000000;
    uh[71] = 1000000;
    uh[72] = 1000000;
    uh[73] = 1000000;
    uh[74] = 1000000;
    uh[75] = 1000000;
    uh[76] = 1000000;
    uh[77] = 1000000;
    uh[78] = 1000000;
    uh[79] = 1000000;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_constraints_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "nl_constr_h_fun_jac",
                                      &capsule->nl_constr_h_fun_jac[i-1]);
        ocp_nlp_constraints_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "nl_constr_h_fun",
                                      &capsule->nl_constr_h_fun[i-1]);
        
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "lh", lh);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "uh", uh);
        
        
    }
    free(luh);








    // set up soft bounds for nonlinear constraints
    int* idxsh = malloc(NSH * sizeof(int));
    idxsh[0] = 0;
    idxsh[1] = 1;
    idxsh[2] = 2;
    idxsh[3] = 3;
    idxsh[4] = 4;
    idxsh[5] = 5;
    idxsh[6] = 6;
    idxsh[7] = 7;
    idxsh[8] = 8;
    idxsh[9] = 9;
    idxsh[10] = 10;
    idxsh[11] = 11;
    idxsh[12] = 12;
    idxsh[13] = 13;
    idxsh[14] = 14;
    idxsh[15] = 15;
    idxsh[16] = 16;
    idxsh[17] = 17;
    idxsh[18] = 18;
    idxsh[19] = 19;
    idxsh[20] = 20;
    idxsh[21] = 21;
    idxsh[22] = 22;
    idxsh[23] = 23;
    idxsh[24] = 24;
    idxsh[25] = 25;
    idxsh[26] = 26;
    idxsh[27] = 27;
    idxsh[28] = 28;
    idxsh[29] = 29;
    idxsh[30] = 30;
    idxsh[31] = 31;
    idxsh[32] = 32;
    idxsh[33] = 33;
    idxsh[34] = 34;
    idxsh[35] = 35;
    idxsh[36] = 36;
    idxsh[37] = 37;
    idxsh[38] = 38;
    idxsh[39] = 39;
    idxsh[40] = 40;
    idxsh[41] = 41;
    idxsh[42] = 42;
    idxsh[43] = 43;
    idxsh[44] = 44;
    idxsh[45] = 45;
    idxsh[46] = 46;
    idxsh[47] = 47;
    idxsh[48] = 48;
    idxsh[49] = 49;
    idxsh[50] = 50;
    idxsh[51] = 51;
    idxsh[52] = 52;
    idxsh[53] = 53;
    idxsh[54] = 54;
    idxsh[55] = 55;
    idxsh[56] = 56;
    idxsh[57] = 57;
    idxsh[58] = 58;
    idxsh[59] = 59;
    idxsh[60] = 60;
    idxsh[61] = 61;
    idxsh[62] = 62;
    idxsh[63] = 63;
    idxsh[64] = 64;
    idxsh[65] = 65;
    idxsh[66] = 66;
    idxsh[67] = 67;
    idxsh[68] = 68;
    idxsh[69] = 69;
    idxsh[70] = 70;
    idxsh[71] = 71;
    idxsh[72] = 72;
    idxsh[73] = 73;
    idxsh[74] = 74;
    idxsh[75] = 75;
    idxsh[76] = 76;
    idxsh[77] = 77;
    idxsh[78] = 78;
    idxsh[79] = 79;
    double* lush = calloc(2*NSH, sizeof(double));
    double* lsh = lush;
    double* ush = lush + NSH;
    ush[0] = 1000000;
    ush[1] = 1000000;
    ush[2] = 1000000;
    ush[3] = 1000000;
    ush[4] = 1000000;
    ush[5] = 1000000;
    ush[6] = 1000000;
    ush[7] = 1000000;
    ush[8] = 1000000;
    ush[9] = 1000000;
    ush[10] = 1000000;
    ush[11] = 1000000;
    ush[12] = 1000000;
    ush[13] = 1000000;
    ush[14] = 1000000;
    ush[15] = 1000000;
    ush[16] = 1000000;
    ush[17] = 1000000;
    ush[18] = 1000000;
    ush[19] = 1000000;
    ush[20] = 1000000;
    ush[21] = 1000000;
    ush[22] = 1000000;
    ush[23] = 1000000;
    ush[24] = 1000000;
    ush[25] = 1000000;
    ush[26] = 1000000;
    ush[27] = 1000000;
    ush[28] = 1000000;
    ush[29] = 1000000;
    ush[30] = 1000000;
    ush[31] = 1000000;
    ush[32] = 1000000;
    ush[33] = 1000000;
    ush[34] = 1000000;
    ush[35] = 1000000;
    ush[36] = 1000000;
    ush[37] = 1000000;
    ush[38] = 1000000;
    ush[39] = 1000000;
    ush[40] = 1000000;
    ush[41] = 1000000;
    ush[42] = 1000000;
    ush[43] = 1000000;
    ush[44] = 1000000;
    ush[45] = 1000000;
    ush[46] = 1000000;
    ush[47] = 1000000;
    ush[48] = 1000000;
    ush[49] = 1000000;
    ush[50] = 1000000;
    ush[51] = 1000000;
    ush[52] = 1000000;
    ush[53] = 1000000;
    ush[54] = 1000000;
    ush[55] = 1000000;
    ush[56] = 1000000;
    ush[57] = 1000000;
    ush[58] = 1000000;
    ush[59] = 1000000;
    ush[60] = 1000000;
    ush[61] = 1000000;
    ush[62] = 1000000;
    ush[63] = 1000000;
    ush[64] = 1000000;
    ush[65] = 1000000;
    ush[66] = 1000000;
    ush[67] = 1000000;
    ush[68] = 1000000;
    ush[69] = 1000000;
    ush[70] = 1000000;
    ush[71] = 1000000;
    ush[72] = 1000000;
    ush[73] = 1000000;
    ush[74] = 1000000;
    ush[75] = 1000000;
    ush[76] = 1000000;
    ush[77] = 1000000;
    ush[78] = 1000000;
    ush[79] = 1000000;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "idxsh", idxsh);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "lsh", lsh);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "ush", ush);
    }
    free(idxsh);
    free(lush);



    /* terminal constraints */

    // set up bounds for last stage
    // x
    int* idxbx_e = malloc(NBXN * sizeof(int));
    idxbx_e[0] = 3;
    idxbx_e[1] = 4;
    idxbx_e[2] = 5;
    double* lubx_e = calloc(2*NBXN, sizeof(double));
    double* lbx_e = lubx_e;
    double* ubx_e = lubx_e + NBXN;
    lbx_e[0] = -0.8;
    ubx_e[0] = 0.8;
    lbx_e[1] = -0.3;
    ubx_e[1] = 0.3;
    lbx_e[2] = -1.2;
    ubx_e[2] = 1.2;
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, N, "idxbx", idxbx_e);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, N, "lbx", lbx_e);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, N, "ubx", ubx_e);
    free(idxbx_e);
    free(lubx_e);



















}


static void robot_obstacle_mpc_acados_create_set_opts(robot_obstacle_mpc_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    void *nlp_opts = capsule->nlp_opts;

    /************************************************
    *  opts
    ************************************************/



    int fixed_hess = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "fixed_hess", &fixed_hess);

    double globalization_fixed_step_length = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "globalization_fixed_step_length", &globalization_fixed_step_length);




    int with_solution_sens_wrt_params = false;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "with_solution_sens_wrt_params", &with_solution_sens_wrt_params);

    int with_value_sens_wrt_params = false;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "with_value_sens_wrt_params", &with_value_sens_wrt_params);

    double solution_sens_qp_t_lam_min = 0.000000001;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "solution_sens_qp_t_lam_min", &solution_sens_qp_t_lam_min);

    int globalization_full_step_dual = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "globalization_full_step_dual", &globalization_full_step_dual);

    // set collocation type (relevant for implicit integrators)
    sim_collocation_type collocation_type = GAUSS_LEGENDRE;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_collocation_type", &collocation_type);

    // set up sim_method_num_steps
    // all sim_method_num_steps are identical
    int sim_method_num_steps = 1;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_num_steps", &sim_method_num_steps);

    // set up sim_method_num_stages
    // all sim_method_num_stages are identical
    int sim_method_num_stages = 4;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_num_stages", &sim_method_num_stages);

    int newton_iter_val = 3;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_newton_iter", &newton_iter_val);

    double newton_tol_val = 0;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_newton_tol", &newton_tol_val);

    // set up sim_method_jac_reuse
    bool tmp_bool = (bool) 0;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_jac_reuse", &tmp_bool);

    double levenberg_marquardt = 0.0001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "levenberg_marquardt", &levenberg_marquardt);

    /* options QP solver */
    int qp_solver_cond_N;const int qp_solver_cond_N_ori = 20;
    qp_solver_cond_N = N < qp_solver_cond_N_ori ? N : qp_solver_cond_N_ori; // use the minimum value here
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_cond_N", &qp_solver_cond_N);
    double reg_epsilon = 0.0001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "reg_epsilon", &reg_epsilon);
    double reg_max_cond_block = 10000000;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "reg_max_cond_block", &reg_max_cond_block);

    double reg_min_epsilon = 0.00000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "reg_min_epsilon", &reg_min_epsilon);

    bool reg_adaptive_eps = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "reg_adaptive_eps", &reg_adaptive_eps);

    int nlp_solver_ext_qp_res = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "ext_qp_res", &nlp_solver_ext_qp_res);

    bool store_iterates = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "store_iterates", &store_iterates);
    int log_primal_step_norm = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "log_primal_step_norm", &log_primal_step_norm);

    int log_dual_step_norm = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "log_dual_step_norm", &log_dual_step_norm);

    double nlp_solver_tol_min_step_norm = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_min_step_norm", &nlp_solver_tol_min_step_norm);
    // set HPIPM mode: should be done before setting other QP solver options
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_hpipm_mode", "BALANCE");



    int qp_solver_t0_init = 2;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_t0_init", &qp_solver_t0_init);




    // set SQP specific options
    double nlp_solver_tol_stat = 0.01;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_stat", &nlp_solver_tol_stat);

    double nlp_solver_tol_eq = 0.01;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_eq", &nlp_solver_tol_eq);

    double nlp_solver_tol_ineq = 0.01;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_ineq", &nlp_solver_tol_ineq);

    double nlp_solver_tol_comp = 0.01;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_comp", &nlp_solver_tol_comp);

    int nlp_solver_max_iter = 50;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "max_iter", &nlp_solver_max_iter);

    // set options for adaptive Levenberg-Marquardt Update
    bool with_adaptive_levenberg_marquardt = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "with_adaptive_levenberg_marquardt", &with_adaptive_levenberg_marquardt);

    double adaptive_levenberg_marquardt_lam = 5;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "adaptive_levenberg_marquardt_lam", &adaptive_levenberg_marquardt_lam);

    double adaptive_levenberg_marquardt_mu_min = 0.0000000000000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "adaptive_levenberg_marquardt_mu_min", &adaptive_levenberg_marquardt_mu_min);

    double adaptive_levenberg_marquardt_mu0 = 0.001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "adaptive_levenberg_marquardt_mu0", &adaptive_levenberg_marquardt_mu0);

    double adaptive_levenberg_marquardt_obj_scalar = 2;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "adaptive_levenberg_marquardt_obj_scalar", &adaptive_levenberg_marquardt_obj_scalar);

    bool eval_residual_at_max_iter = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "eval_residual_at_max_iter", &eval_residual_at_max_iter);

    // QP scaling
    double qpscaling_ub_max_abs_eig = 100000;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qpscaling_ub_max_abs_eig", &qpscaling_ub_max_abs_eig);

    double qpscaling_lb_norm_inf_grad_obj = 0.0001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qpscaling_lb_norm_inf_grad_obj", &qpscaling_lb_norm_inf_grad_obj);

    qpscaling_scale_objective_type qpscaling_scale_objective = NO_OBJECTIVE_SCALING;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qpscaling_scale_objective", &qpscaling_scale_objective);

    ocp_nlp_qpscaling_constraint_type qpscaling_scale_constraints = NO_CONSTRAINT_SCALING;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qpscaling_scale_constraints", &qpscaling_scale_constraints);

    // NLP QP tol strategy
    ocp_nlp_qp_tol_strategy_t nlp_qp_tol_strategy = FIXED_QP_TOL;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "nlp_qp_tol_strategy", &nlp_qp_tol_strategy);

    double nlp_qp_tol_reduction_factor = 0.1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "nlp_qp_tol_reduction_factor", &nlp_qp_tol_reduction_factor);

    double nlp_qp_tol_safety_factor = 0.1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "nlp_qp_tol_safety_factor", &nlp_qp_tol_safety_factor);

    double nlp_qp_tol_min_stat = 0.000000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "nlp_qp_tol_min_stat", &nlp_qp_tol_min_stat);

    double nlp_qp_tol_min_eq = 0.0000000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "nlp_qp_tol_min_eq", &nlp_qp_tol_min_eq);

    double nlp_qp_tol_min_ineq = 0.0000000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "nlp_qp_tol_min_ineq", &nlp_qp_tol_min_ineq);

    double nlp_qp_tol_min_comp = 0.00000000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "nlp_qp_tol_min_comp", &nlp_qp_tol_min_comp);

    bool with_anderson_acceleration = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "with_anderson_acceleration", &with_anderson_acceleration);

    double anderson_activation_threshold = 10;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "anderson_activation_threshold", &anderson_activation_threshold);

    int qp_solver_iter_max = 100;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_iter_max", &qp_solver_iter_max);


    double qp_solver_tol_stat = 0.001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_tol_stat", &qp_solver_tol_stat);
    double qp_solver_tol_eq = 0.001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_tol_eq", &qp_solver_tol_eq);
    double qp_solver_tol_ineq = 0.001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_tol_ineq", &qp_solver_tol_ineq);
    double qp_solver_tol_comp = 0.001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_tol_comp", &qp_solver_tol_comp);

    int print_level = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "print_level", &print_level);
    int qp_solver_cond_ric_alg = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_cond_ric_alg", &qp_solver_cond_ric_alg);

    int qp_solver_ric_alg = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_ric_alg", &qp_solver_ric_alg);


    int ext_cost_num_hess = 0;
}


/**
 * Internal function for robot_obstacle_mpc_acados_create: step 7
 */
void robot_obstacle_mpc_acados_set_nlp_out(robot_obstacle_mpc_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_out* nlp_out = capsule->nlp_out;
    ocp_nlp_in* nlp_in = capsule->nlp_in;

    // initialize primal solution
    double* xu0 = calloc(NX+NU, sizeof(double));
    double* x0 = xu0;

    // initialize with x0


    double* u0 = xu0 + NX;

    for (int i = 0; i < N; i++)
    {
        // x0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "x", x0);
        // u0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "u", u0);
    }
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, N, "x", x0);
    free(xu0);
}


/**
 * Internal function for robot_obstacle_mpc_acados_create: step 9
 */
int robot_obstacle_mpc_acados_create_precompute(robot_obstacle_mpc_solver_capsule* capsule) {
    int status = ocp_nlp_precompute(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    if (status != ACADOS_SUCCESS) {
        printf("\nocp_nlp_precompute failed!\n\n");
        exit(1);
    }

    return status;
}


int robot_obstacle_mpc_acados_create_with_discretization(robot_obstacle_mpc_solver_capsule* capsule, int N, double* new_time_steps)
{
    // If N does not match the number of shooting intervals used for code generation, new_time_steps must be given.
    if (N != ROBOT_OBSTACLE_MPC_N && !new_time_steps) {
        fprintf(stderr, "robot_obstacle_mpc_acados_create_with_discretization: new_time_steps is NULL " \
            "but the number of shooting intervals (= %d) differs from the number of " \
            "shooting intervals (= %d) during code generation! Please provide a new vector of time_stamps!\n", \
             N, ROBOT_OBSTACLE_MPC_N);
        return 1;
    }

    // number of expected runtime parameters
    capsule->nlp_np = NP;

    // 1) create and set nlp_solver_plan; create nlp_config
    capsule->nlp_solver_plan = ocp_nlp_plan_create(N);
    robot_obstacle_mpc_acados_create_set_plan(capsule->nlp_solver_plan, N);
    capsule->nlp_config = ocp_nlp_config_create(*capsule->nlp_solver_plan);

    // 2) create and set dimensions
    capsule->nlp_dims = robot_obstacle_mpc_acados_create_setup_dimensions(capsule);

    // 3) create and set nlp_opts
    capsule->nlp_opts = ocp_nlp_solver_opts_create(capsule->nlp_config, capsule->nlp_dims);
    robot_obstacle_mpc_acados_create_set_opts(capsule);

    // 4) create and set nlp_out
    // 4.1) nlp_out
    capsule->nlp_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    // 4.2) sens_out
    capsule->sens_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    robot_obstacle_mpc_acados_set_nlp_out(capsule);

    // 5) create nlp_in
    capsule->nlp_in = ocp_nlp_in_create(capsule->nlp_config, capsule->nlp_dims);

    // 6) setup functions, nlp_in and default parameters
    robot_obstacle_mpc_acados_create_setup_functions(capsule);
    robot_obstacle_mpc_acados_setup_nlp_in(capsule, N, new_time_steps);
    robot_obstacle_mpc_acados_create_set_default_parameters(capsule);

    // 7) create solver
    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts, capsule->nlp_in);


    // 8) do precomputations
    int status = robot_obstacle_mpc_acados_create_precompute(capsule);

    return status;
}

/**
 * This function is for updating an already initialized solver with a different number of qp_cond_N. It is useful for code reuse after code export.
 */
int robot_obstacle_mpc_acados_update_qp_solver_cond_N(robot_obstacle_mpc_solver_capsule* capsule, int qp_solver_cond_N)
{
    // 1) destroy solver
    ocp_nlp_solver_destroy(capsule->nlp_solver);

    // 2) set new value for "qp_cond_N"
    const int N = capsule->nlp_solver_plan->N;
    if(qp_solver_cond_N > N)
        printf("Warning: qp_solver_cond_N = %d > N = %d\n", qp_solver_cond_N, N);
    ocp_nlp_solver_opts_set(capsule->nlp_config, capsule->nlp_opts, "qp_cond_N", &qp_solver_cond_N);

    // 3) continue with the remaining steps from robot_obstacle_mpc_acados_create_with_discretization(...):
    // -> 8) create solver
    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts, capsule->nlp_in);

    // -> 9) do precomputations
    int status = robot_obstacle_mpc_acados_create_precompute(capsule);
    return status;
}


int robot_obstacle_mpc_acados_reset(robot_obstacle_mpc_solver_capsule* capsule, int reset_qp_solver_mem)
{

    // set initialization to all zeros

    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_out* nlp_out = capsule->nlp_out;
    ocp_nlp_in* nlp_in = capsule->nlp_in;
    ocp_nlp_solver* nlp_solver = capsule->nlp_solver;

    double* buffer = calloc(NX+NU+NZ+2*NS+2*NSN+2*NS0+NBX+NBU+NG+NH+NPHI+NBX0+NBXN+NHN+NH0+NPHIN+NGN, sizeof(double));

    for(int i=0; i<N+1; i++)
    {
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "x", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "u", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "sl", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "su", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "lam", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "z", buffer);
        if (i<N)
        {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "pi", buffer);
        }
    }
    // get qp_status: if NaN -> reset memory
    int qp_status;
    ocp_nlp_get(capsule->nlp_solver, "qp_status", &qp_status);
    if (reset_qp_solver_mem || (qp_status == 3))
    {
        // printf("\nin reset qp_status %d -> resetting QP memory\n", qp_status);
        ocp_nlp_solver_reset_qp_memory(nlp_solver, nlp_in, nlp_out);
    }

    free(buffer);
    return 0;
}




int robot_obstacle_mpc_acados_update_params(robot_obstacle_mpc_solver_capsule* capsule, int stage, double *p, int np)
{
    int solver_status = 0;

    int casadi_np = 30;
    if (casadi_np != np) {
        printf("acados_update_params: trying to set %i parameters for external functions."
            " External function has %i parameters. Exiting.\n", np, casadi_np);
        exit(1);
    }
    ocp_nlp_in_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, stage, "parameter_values", p);

    return solver_status;
}


int robot_obstacle_mpc_acados_update_params_sparse(robot_obstacle_mpc_solver_capsule * capsule, int stage, int *idx, double *p, int n_update)
{
    ocp_nlp_in_set_params_sparse(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, stage, idx, p, n_update);

    return 0;
}


int robot_obstacle_mpc_acados_set_p_global_and_precompute_dependencies(robot_obstacle_mpc_solver_capsule* capsule, double* data, int data_len)
{

    // printf("No global_data, robot_obstacle_mpc_acados_set_p_global_and_precompute_dependencies does nothing.\n");
    return 0;
}




int robot_obstacle_mpc_acados_solve(robot_obstacle_mpc_solver_capsule* capsule)
{
    // solve NLP
    int solver_status = ocp_nlp_solve(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    return solver_status;
}



int robot_obstacle_mpc_acados_setup_qp_matrices_and_factorize(robot_obstacle_mpc_solver_capsule* capsule)
{
    int solver_status = ocp_nlp_setup_qp_matrices_and_factorize(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    return solver_status;
}






int robot_obstacle_mpc_acados_free(robot_obstacle_mpc_solver_capsule* capsule)
{
    // before destroying, keep some info
    const int N = capsule->nlp_solver_plan->N;
    // free memory
    ocp_nlp_solver_opts_destroy(capsule->nlp_opts);
    ocp_nlp_in_destroy(capsule->nlp_in);
    ocp_nlp_out_destroy(capsule->nlp_out);
    ocp_nlp_out_destroy(capsule->sens_out);
    ocp_nlp_solver_destroy(capsule->nlp_solver);
    ocp_nlp_dims_destroy(capsule->nlp_dims);
    ocp_nlp_config_destroy(capsule->nlp_config);
    ocp_nlp_plan_destroy(capsule->nlp_solver_plan);

    /* free external function */
    // dynamics
    for (int i = 0; i < N; i++)
    {
        external_function_external_param_casadi_free(&capsule->expl_vde_forw[i]);
        
        external_function_external_param_casadi_free(&capsule->expl_ode_fun[i]);
        external_function_external_param_casadi_free(&capsule->expl_vde_adj[i]);
    }
    free(capsule->expl_vde_adj);
    free(capsule->expl_vde_forw);
    
    free(capsule->expl_ode_fun);

    // cost

    // constraints
    for (int i = 0; i < N-1; i++)
    {
        external_function_external_param_casadi_free(&capsule->nl_constr_h_fun_jac[i]);
        external_function_external_param_casadi_free(&capsule->nl_constr_h_fun[i]);
    }
    free(capsule->nl_constr_h_fun_jac);
    free(capsule->nl_constr_h_fun);



    return 0;
}


void robot_obstacle_mpc_acados_print_stats(robot_obstacle_mpc_solver_capsule* capsule)
{
    int nlp_iter, stat_m, stat_n, tmp_int;
    ocp_nlp_get(capsule->nlp_solver, "nlp_iter", &nlp_iter);
    ocp_nlp_get(capsule->nlp_solver, "stat_n", &stat_n);
    ocp_nlp_get(capsule->nlp_solver, "stat_m", &stat_m);


    int stat_n_max = 16;
    if (stat_n > stat_n_max)
    {
        printf("stat_n_max = %d is too small, increase it in the template!\n", stat_n_max);
        exit(1);
    }
    double stat[816];
    ocp_nlp_get(capsule->nlp_solver, "statistics", stat);

    int nrow = nlp_iter+1 < stat_m ? nlp_iter+1 : stat_m;


    printf("iter\tres_stat\tres_eq\t\tres_ineq\tres_comp\tqp_stat\tqp_iter\talpha");
    if (stat_n > 8)
        printf("\t\tqp_res_stat\tqp_res_eq\tqp_res_ineq\tqp_res_comp");
    printf("\n");
    for (int i = 0; i < nrow; i++)
    {
        for (int j = 0; j < stat_n + 1; j++)
        {
            if (j == 0 || j == 5 || j == 6)
            {
                tmp_int = (int) stat[i + j * nrow];
                printf("%d\t", tmp_int);
            }
            else
            {
                printf("%e\t", stat[i + j * nrow]);
            }
        }
        printf("\n");
    }
}

int robot_obstacle_mpc_acados_custom_update(robot_obstacle_mpc_solver_capsule* capsule, double* data, int data_len)
{
    (void)capsule;
    (void)data;
    (void)data_len;
    printf("\ndummy function that can be called in between solver calls to update parameters or numerical data efficiently in C.\n");
    printf("nothing set yet..\n");
    return 1;

}



ocp_nlp_in *robot_obstacle_mpc_acados_get_nlp_in(robot_obstacle_mpc_solver_capsule* capsule) { return capsule->nlp_in; }
ocp_nlp_out *robot_obstacle_mpc_acados_get_nlp_out(robot_obstacle_mpc_solver_capsule* capsule) { return capsule->nlp_out; }
ocp_nlp_out *robot_obstacle_mpc_acados_get_sens_out(robot_obstacle_mpc_solver_capsule* capsule) { return capsule->sens_out; }
ocp_nlp_solver *robot_obstacle_mpc_acados_get_nlp_solver(robot_obstacle_mpc_solver_capsule* capsule) { return capsule->nlp_solver; }
ocp_nlp_config *robot_obstacle_mpc_acados_get_nlp_config(robot_obstacle_mpc_solver_capsule* capsule) { return capsule->nlp_config; }
void *robot_obstacle_mpc_acados_get_nlp_opts(robot_obstacle_mpc_solver_capsule* capsule) { return capsule->nlp_opts; }
ocp_nlp_dims *robot_obstacle_mpc_acados_get_nlp_dims(robot_obstacle_mpc_solver_capsule* capsule) { return capsule->nlp_dims; }
ocp_nlp_plan_t *robot_obstacle_mpc_acados_get_nlp_plan(robot_obstacle_mpc_solver_capsule* capsule) { return capsule->nlp_solver_plan; }

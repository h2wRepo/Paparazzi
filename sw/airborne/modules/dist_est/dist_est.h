/*
 * Copyright (C) 2016 Hann Woei Ho
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/dist_est/dist_est.h
 * @Height estimation using flow divergence and control input
 */

#ifndef DIST_EST_H_
#define DIST_EST_H_

#include "std.h"

/* The divergence landing */
struct Div_landing_t {
  float div_pgain;        ///< The divergence P gain on the err_div
  float div_igain;        ///< The divergence I gain on the err_div_int
  float div_dgain;        ///< The divergence D gain on the err_div_int
  float nominal_throttle; ///< The nominal throttle
  float desired_div;      ///< The desired divergence
  int32_t controller;	  ///< The controller switch
  float cov_div;		  ///< The covariance of divergence
  float div;              ///< The divergence
  float div_f;            ///< The filtered divergence (low-pass)
  float ground_div;       ///< The ground divergence
  float agl;			  ///< The sonar height
  float gps_z;			  ///< The height
  float vel_z;			  ///< The velocity
  float accel_z;		  ///< The acceleration
  float z_sp;			  ///< The height setpoint
  float err_Z;			  ///< The height error
  float err_Vz;			  ///< The velocity error
  float z_sum_err;		  ///< The sum of height error
  int32_t thrust;		  ///< The thrust
  float fps;			  ///< The frame rate
  float alpha;			  ///< The filter update
  float stamp;			  ///< The timestamp
  float t_interval_sp;	  ///< The time interval
};
extern struct Div_landing_t Div_landing;

// use the hover mode for horizontal control
//#define GUIDANCE_H_MODE_MODULE_SETTING GUIDANCE_H_MODE_HOVER

// use guidance from module
#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_MODULE

// use vertical guidance loops from module
extern void guidance_v_module_init(void);
extern void guidance_v_module_enter(void);
extern void guidance_v_module_run(bool_t in_flight);

#endif /* DIST_EST_H_ */

/*
 * urdf_parser.h
 *
 *  Created on: 14.05.2026
 *      Author: jan
 */

#ifndef MAIN_URDF_PARSER_H_
#define MAIN_URDF_PARSER_H_

#include <stdbool.h>

typedef struct {
    float wheel_radius_m;
    float wheel_diameter_m;
    float wheel_separation_m;
    bool valid;
} wheel_geometry_t;

bool urdf_parse_wheel_geometry(const char *urdf, wheel_geometry_t *geometry);




#endif /* MAIN_URDF_PARSER_H_ */

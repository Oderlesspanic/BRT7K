/*
 * urdf_parser.c
 *
 *  Created on: 14.05.2026
 *      Author: jan
 */

 #include "urdf_parser.h"

 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>
 #include <math.h>

 static bool parse_joint_y_offset(
     const char *urdf,
     const char *joint_name,
     float *y_out)
 {
     char search[128];

     snprintf(search, sizeof(search), "<joint name=\"%s\"", joint_name);

     const char *joint = strstr(urdf, search);
     if (joint == NULL) return false;

     const char *origin = strstr(joint, "<origin");
     if (origin == NULL) return false;

     const char *xyz = strstr(origin, "xyz=\"");
     if (xyz == NULL) return false;

     xyz += strlen("xyz=\"");

     float x = 0.0f;
     float y = 0.0f;
     float z = 0.0f;

     if (sscanf(xyz, "%f %f %f", &x, &y, &z) != 3) {
         return false;
     }

     *y_out = y;
     return true;
 }

 static bool parse_link_cylinder_radius(
     const char *urdf,
     const char *link_name,
     float *radius_out)
 {
     char search[128];

     snprintf(search, sizeof(search), "<link name=\"%s\"", link_name);

     const char *link = strstr(urdf, search);
     if (link == NULL) return false;

     const char *cylinder = strstr(link, "<cylinder");
     if (cylinder == NULL) return false;

     const char *radius = strstr(cylinder, "radius=\"");
     if (radius == NULL) return false;

     radius += strlen("radius=\"");

     *radius_out = strtof(radius, NULL);
     return true;
 }

 bool urdf_parse_wheel_geometry(const char *urdf, wheel_geometry_t *geometry)
 {
     if (urdf == NULL || geometry == NULL) {
         return false;
     }

     float radius = 0.0f;
     float left_y = 0.0f;
     float right_y = 0.0f;

     bool radius_ok = parse_link_cylinder_radius(
         urdf,
         "left_wheel_link",
         &radius
     );

     bool left_ok = parse_joint_y_offset(
         urdf,
         "left_wheel_link_joint",
         &left_y
     );

     bool right_ok = parse_joint_y_offset(
         urdf,
         "right_wheel_link_joint",
         &right_y
     );

     if (!radius_ok || !left_ok || !right_ok) {
         geometry->valid = false;
         return false;
     }

     geometry->wheel_radius_m = radius;
     geometry->wheel_diameter_m = 2.0f * radius;
     geometry->wheel_separation_m = fabsf(left_y - right_y);
     geometry->valid = true;

     return true;
 }

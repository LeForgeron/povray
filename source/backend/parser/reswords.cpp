/*******************************************************************************
 * reswords.cpp
 *
 * This file contains the list of reserved words as a global array. It is
 * kept separate from the parser to allow it to be linked in with GUI's that
 * may not link the core rendering code.
 *
 * ---------------------------------------------------------------------------
 * Persistence of Vision Ray Tracer ('POV-Ray') version 3.7.
 * Copyright 1991-2013 Persistence of Vision Raytracer Pty. Ltd.
 *
 * POV-Ray is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * POV-Ray is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * ---------------------------------------------------------------------------
 * POV-Ray is based on the popular DKB raytracer version 2.12.
 * DKBTrace was originally written by David K. Buck.
 * DKBTrace Ver 2.0-2.12 were written by David K. Buck & Aaron A. Collins.
 * ---------------------------------------------------------------------------
 * $File: //depot/public/povray/3.x/source/backend/parser/reswords.cpp $
 * $Revision: #1 $
 * $Change: 6069 $
 * $DateTime: 2013/11/06 11:59:40 $
 * $Author: chrisc $
 *******************************************************************************/

// frame.h must always be the first POV file included (pulls in platform config)
#include "backend/frame.h"

#include "backend/parser/reswords.h"

// this must be the last file included
#include "base/povdebug.h"

namespace pov
{

/*
 * Here are the reserved words.  If you need to add new words,
 * be sure to declare them in reswords.h
 */
const RESERVED_WORD Reserved_Words[LAST_TOKEN] = {
	{AA_THRESHOLD_TOKEN, "aa_threshold"},
	{AA_LEVEL_TOKEN, "aa_level"},
	{ABSORPTION_TOKEN, "absorption"},
	{ABS_TOKEN, "abs"},
	{ACCURACY_TOKEN, "accuracy"},
	{ACOSH_TOKEN, "acosh"},
	{ACOS_TOKEN, "acos"},
	{ADAPTIVE_TOKEN, "adaptive"},
	{ADC_BAILOUT_TOKEN, "adc_bailout"},
	{AGATE_TOKEN, "agate"},
	{AGATE_TURB_TOKEN, "agate_turb"},
	{AITOFF_HAMMER_TOKEN, "aitoff_hammer"},
	{AKIMA_SPLINE_TOKEN, "akima_spline"},
	{ALBEDO_TOKEN, "albedo"},
	{ALL_INTERSECTIONS_TOKEN, "all_intersections"},
	{ALL_TOKEN, "all"},
	{ALPHA_TOKEN, "alpha"},
	{ALBINOS_TOKEN, "albinos"},
	{AMOUNT_TOKEN, "amount"},
	{ALWAYS_SAMPLE_TOKEN, "always_sample"},
	{ALTITUDE_TOKEN, "altitude"},
	{AMBIENT_LIGHT_TOKEN, "ambient_light"},
	{AMBIENT_TOKEN, "ambient"},
	{AMPERSAND_TOKEN, "&"},
	{ANGLE_TOKEN, "angle"},
	{ANISOTROPY_TOKEN, "anisotropy"},
	{AOI_TOKEN,"aoi"},
	{APERTURE_TOKEN, "aperture"},
	{APPEND_TOKEN, "append"},
	{ARC_ANGLE_TOKEN, "arc_angle"},
	{AREA_ILLUMINATION_TOKEN, "area_illumination"},
	{AREA_LIGHT_TOKEN, "area_light"},
	{ARRAY_ID_TOKEN, "array identifier"},
	{ARRAY_TOKEN, "array"},
	{ASCII_TOKEN, "ascii"},
	{ASC_TOKEN, "asc"},
	{ASINH_TOKEN, "asinh"},
	{ASIN_TOKEN, "asin"},
	{ASSUMED_GAMMA_TOKEN, "assumed_gamma"},
	{ATAN2_TOKEN, "atan2"},
	{ATANH_TOKEN, "atanh"},
	{ATAN_TOKEN, "atan"},
	{AT_TOKEN, "@"},
	{AUTOSTOP_TOKEN, "autostop"},
	{AVERAGE_TOKEN, "average"},
	{BACKGROUND_TOKEN, "background"},
	{BALTHASART_TOKEN, "balthasart"},
	{BACK_QUOTE_TOKEN, "`"},
	{BACK_SLASH_TOKEN, "\\"},
	{BAR_TOKEN, "|"},
	{BASIC_X_SPLINE_TOKEN, "basic_x_spline"},
	{BEHRMANN_TOKEN, "behrmann"},
	{BEND_TOKEN, "bend"},
	{BEZIER_SPLINE_TOKEN, "bezier_spline"},
	{BIAS_TOKEN, "bias"},
	{BICUBIC_PATCH_TOKEN, "bicubic_patch"},
	{BINARY_TOKEN,"binary"},
	{INBOUND_TOKEN, "inbound"},
	{BITWISE_AND_TOKEN, "bitwise_and"},
	{BITWISE_OR_TOKEN, "bitwise_or"},
	{BITWISE_XOR_TOKEN, "bitwise_xor"},
	{BLACK_HOLE_TOKEN, "black_hole"},
	{BLOB_TOKEN, "blob"},
	{BLUE_TOKEN, "blue"},
	{BLUR_SAMPLES_TOKEN, "blur_samples"},
	{BMP_TOKEN, "bmp"},
	{BOKEH_TOKEN, "bokeh"},
	{BOUNDED_BY_TOKEN, "bounded_by"},
	{BOURKE_TOKEN, "bourke"},
	{OUTBOUND_TOKEN, "outbound"},
	{BOXED_TOKEN, "boxed"},
	{BOX_TOKEN, "box"},
	{BOZO_TOKEN, "bozo"},
	{BREAK_TOKEN, "break"},
	{BRICK_SIZE_TOKEN, "brick_size"},
	{BRICK_TOKEN, "brick"},
	{BRIGHTNESS_TOKEN, "brightness" },
	{BRILLIANCE_TOKEN, "brilliance"},
	{BUMPS_TOKEN, "bumps"},
	{BUMP_MAP_TOKEN, "bump_map"},
	{BUMP_SIZE_TOKEN, "bump_size"},
	{B_SPLINE_TOKEN, "b_spline"},
	{CAMERA_ID_TOKEN, "camera identifier"},
	{CAMERA_TOKEN, "camera"},
	{CAMERA_DIRECTION_TOKEN, "camera_direction"},
	{CAMERA_LOCATION_TOKEN, "camera_location"},
	{CAMERA_RIGHT_TOKEN, "camera_right"},
	{CAMERA_TYPE_TOKEN, "camera_type"},
	{CAMERA_UP_TOKEN, "camera_up"},
	{CASE_TOKEN, "case"},
	{CAUSTICS_TOKEN, "caustics"},
	{CEIL_TOKEN, "ceil"},
	{CELLS_TOKEN, "cells"},
	{CHARSET_TOKEN, "charset"},
	{CHECKER_TOKEN, "checker"},
	{CHILD_TOKEN, "child"},
	{CHILDREN_TOKEN, "children"},
	{CHR_TOKEN, "chr"},
	{CIRCULAR_TOKEN,"circular"},
	{CLIPPED_BY_TOKEN, "clipped_by"},
	{CLOCK_ON_TOKEN, "clock_on"},
	{CLOCK_TOKEN, "clock"},
	{COLLECT_TOKEN, "collect"},
	{COLON_TOKEN, ":"},
	{COLOUR_ID_TOKEN, "colour identifier"},
	{COLOUR_KEY_TOKEN, "color keyword"},
	{COLOUR_MAP_ID_TOKEN, "colour map identifier"},
	{COLOUR_MAP_TOKEN, "color_map"},
	{COLOUR_MAP_TOKEN, "colour_map"},
	{COLOUR_SPACE_TOKEN, "color_space"},
	{COLOUR_SPACE_TOKEN, "colour_space"},
	{COLOUR_TOKEN, "color"},
	{COLOUR_TOKEN, "colour"},
	{COMMA_TOKEN, ", "},
	{COMPONENT_TOKEN, "component"},
	{COMPOSITE_TOKEN, "composite"},
	{CONCAT_TOKEN, "concat"},
	{CONE_TOKEN, "cone"},
	{CONFIDENCE_TOKEN, "confidence"},
	{CONIC_SWEEP_TOKEN, "conic_sweep"},
	{CONSERVE_ENERGY_TOKEN, "conserve_energy"},
	{CONTAINED_BY_TOKEN,"contained_by"},
	{CONTINUITY_TOKEN, "continuity"},
	{CONTROL0_TOKEN, "control0"},
	{CONTROL1_TOKEN, "control1"},
	{COORDS_TOKEN, "coords" },
	{COSH_TOKEN, "cosh"},
	{COS_TOKEN, "cos"},
	{COUNT_TOKEN, "count" },
	{CRACKLE_TOKEN, "crackle"},
	{CRAND_TOKEN, "crand"},
	{CRISTAL_TOKEN, "cristal"},
	{CUBE_TOKEN, "cube"},
	{CUBICLE_TOKEN, "cubicle"},
	{CUBIC_SPLINE_TOKEN, "cubic_spline"},
	{CUBIC_TOKEN, "cubic"},
	{CUBIC_WAVE_TOKEN, "cubic_wave"},
	{CUTAWAY_TEXTURES_TOKEN, "cutaway_textures"},
	{CYLINDER_TOKEN, "cylinder"},
	{CYLINDRICAL_TOKEN, "cylindrical"},
	{DASH_TOKEN, "-"},
	{DATETIME_TOKEN, "datetime"},
	{DEBUG_TOKEN, "debug"},
	{DEBUG_TAG_TOKEN, "dtag"},
	{DECLARE_TOKEN, "declare"},
	{DEFAULT_TOKEN, "default"},
	{DEFINED_TOKEN, "defined"},
	{DEGREES_TOKEN, "degrees"},
	{DENSITY_FILE_TOKEN, "density_file"},
	{DENSITY_ID_TOKEN, "density identifier"},
	{DENSITY_MAP_ID_TOKEN, "density_map identifier"},
	{DENSITY_MAP_TOKEN, "density_map"},
	{DENSITY_TOKEN, "density"},
	{DENTS_TOKEN, "dents"},
	{DEPRECATED_TOKEN, "deprecated"},
	{DF3_TOKEN, "df3"},
	{DIFFERENCE_TOKEN, "difference"},
	{DIFFUSE_TOKEN, "diffuse"},
	{DIMENSIONS_TOKEN, "dimensions"},
	{DIMENSION_SIZE_TOKEN, "dimension_size"},
	{DIRECTION_TOKEN, "direction"},
	{DISC_TOKEN, "disc"},
	{DISPLACE_TOKEN, "displace"},
	{DISPERSION_TOKEN, "dispersion"},
	{DISPERSION_SAMPLES_TOKEN, "dispersion_samples"},
#if 0 // [CLi] the distance_maximum token is perfectly obsolete
	{DISTANCE_MAXIMUM_TOKEN, "distance_maximum" },
#endif
	{DISTANCE_TOKEN, "distance"},
	{DIST_EXP_TOKEN, "dist_exp"},
	{DIV_TOKEN, "div"},
	{DOLLAR_TOKEN, "$"},
	{DOUBLE_ILLUMINATE_TOKEN, "double_illuminate"},
	{ECCENTRICITY_TOKEN, "eccentricity"},
	{ECKERT4_TOKEN, "eckert_iv"},
	{ECKERT6_TOKEN, "eckert_vi"},
	{EDWARDS_TOKEN, "edwards"},
	{ELSE_TOKEN, "else"},
	{ELSEIF_TOKEN, "elseif"},
	{EMISSION_TOKEN, "emission"},
	{EMPTY_ARRAY_TOKEN, "empty array"},
	{END_OF_FILE_TOKEN, "End of File"},
	{END_TOKEN, "end"},
	{EQUALS_TOKEN, "="},
	{ERROR_BOUND_TOKEN, "error_bound" },
	{ERROR_TOKEN, "error"},
	{EVALUATE_TOKEN, "evaluate"},
	{EXCLAMATION_TOKEN, "!"},
	{EXPAND_THRESHOLDS_TOKEN, "expand_thresholds"},
	{EXP_TOKEN, "exp"},
	{EXPONENT_TOKEN, "exponent"},
	{EXR_TOKEN, "exr"},
	{EXTENDED_X_SPLINE_TOKEN, "extended_x_spline"},
	{EXTERIOR_TOKEN, "exterior"},
	{EXTINCTION_TOKEN, "extinction"},
	{FACETS_TOKEN, "facets"},
	{FACE_INDICES_TOKEN, "face_indices"},
	{FADE_COLOUR_TOKEN, "fade_colour"},
	{FADE_COLOUR_TOKEN, "fade_color"},
	{FADE_DISTANCE_TOKEN, "fade_distance"},
	{FADE_POWER_TOKEN, "fade_power"},
	{FALLOFF_ANGLE_TOKEN, "falloff_angle"},
	{FALLOFF_TOKEN, "falloff"},
	{FALSE_TOKEN, "false"},
	{FCLOSE_TOKEN, "fclose"},
	{FILE_EXISTS_TOKEN, "file_exists"},
	{FILE_ID_TOKEN, "file identifier"},
	{FILL_LIGHT_TOKEN, "shadowless"},
	{FILTER_TOKEN, "filter"},
	{FINISH_ID_TOKEN, "finish identifier"},
	{FINISH_TOKEN, "finish"},
	{FISHEYE_TOKEN, "fisheye"},
	{FIXED_TOKEN, "fixed"},
	{FLATNESS_TOKEN, "flatness"},
	{FLIP_TOKEN, "flip"},
	{FLOAT_FUNCT_TOKEN, "float function"},
	{FLOAT_ID_TOKEN, "float identifier"},
	{FLOAT_TOKEN, "float constant"},
	{FLOOR_TOKEN, "floor"},
	{FOCAL_POINT_TOKEN, "focal_point"},
	{FOG_ALT_TOKEN, "fog_alt"},
	{FOG_ID_TOKEN, "fog identifier"},
	{FOG_OFFSET_TOKEN, "fog_offset"},
	{FOG_TOKEN, "fog"},
	{FOG_TYPE_TOKEN, "fog_type"},
	{FOPEN_TOKEN, "fopen"},
	{FOR_TOKEN, "for"},
	{FORM_TOKEN, "form"},
	{FREEDOM_DEGREE_TOKEN, "freedom_degree"},
	{FREQUENCY_TOKEN, "frequency"},
	{FRESNEL_TOKEN, "fresnel"},
	{FUNCTION_TOKEN, "function"},
	{FUNCT_ID_TOKEN, "function identifier"},
	{GALL_TOKEN, "gall"},
	{GALLEY_TOKEN, "galley"},
	{GAMMA_TOKEN, "gamma"},
	{GATHER_TOKEN, "gather"},
	{GENERAL_X_SPLINE_TOKEN, "general_x_spline"},
	{GET_NORMALS_AMOUNT_TOKEN, "get_normal_count"},
	{GET_NORMAL_INDICES_TOKEN, "get_normal_indices"},
	{GET_NORMAL_TOKEN, "get_normal"},
	{GET_TRIANGLES_AMOUNT_TOKEN, "get_triangle_count"},
	{GET_VERTEX_INDICES_TOKEN, "get_vertex_indices"},
	{GET_VERTEX_TOKEN, "get_vertex"},
	{GET_VERTICES_AMOUNT_TOKEN, "get_vertex_count"},
	{GIF_TOKEN, "gif"},
	{GLOBAL_LIGHTS_TOKEN, "global_lights"},
	{GLOBAL_SETTINGS_TOKEN, "global_settings" },
#ifdef GLOBAL_PHOTONS
	{GLOBAL_TOKEN, "global"},
#endif
	{GRADIENT_TOKEN, "gradient"},
	{GRANITE_TOKEN, "granite"},
	{GRAY_TOKEN, "gray"},
	{GRAY_THRESHOLD_TOKEN, "gray_threshold" },
	{GREEN_TOKEN, "green"},
	{GRID_TOKEN, "grid"},
	{GTS_LOAD_TOKEN, "gts_load"},
	{GTS_SAVE_TOKEN, "gts_save"},
	{HASH_TOKEN, "#"},
	{HAT_TOKEN, "^"},
	{HDR_TOKEN, "hdr"},
	{HELLER_TOKEN, "heller"},
	{HEIGHT_FIELD_TOKEN, "height_field"},
	{HEXAGON_TOKEN, "hexagon"},
	{HF_GRAY_16_TOKEN, "hf_gray_16" },
	{HIERARCHY_TOKEN, "hierarchy"},
	{HOBO_DYER_TOKEN, "hobo_dyer"},
	{HOLLOW_TOKEN, "hollow"},
	{HSL_TOKEN, "hsl"},
	{HSV_TOKEN, "hsv"},
	{HYPERCOMPLEX_TOKEN, "hypercomplex"},
	{ICOSA_TOKEN, "icosa"},
	{IDENTIFIER_TOKEN, "undeclared identifier"},
	{INTERIOR_TEXTURE_TOKEN, "interior_texture"},
	{INTERNAL_TOKEN, "internal"},
	{IFDEF_TOKEN, "ifdef"},
	{IFF_TOKEN, "iff"},
	{IFNDEF_TOKEN, "ifndef"},
	{IF_TOKEN, "if"},
	{IMAGE_MAP_TOKEN, "image_map"},
	{IMAGE_PATTERN_TOKEN, "image_pattern"},
	{IMPORTANCE_TOKEN, "importance"},
	{INCLUDE_TOKEN, "include"},
	{INNER_TOKEN,"inner"},
	{INSIDE_TOKEN, "inside"},
	{INSIDE_POINT_TOKEN, "inside_point"},
	{INSIDE_VECTOR_TOKEN, "inside_vector"},
	{INTERIOR_ID_TOKEN, "interior identifier"},
	{INTERIOR_TOKEN, "interior"},
	{INTERMERGE_TOKEN, "intermerge"},
	{INTERPOLATE_TOKEN, "interpolate"},
	{INTERSECTION_TOKEN, "intersection"},
	{INTERUNION_TOKEN, "interunion"},
	{INTERVALS_TOKEN, "intervals"},
	{INT_TOKEN, "int"},
	{INVERSE_TOKEN, "inverse"},
	{IOR_TOKEN, "ior"},
	{IRID_TOKEN, "irid"},
	{IRID_WAVELENGTH_TOKEN, "irid_wavelength"},
	{IS_SMOOTH_TRIANGLE_TOKEN, "is_smooth_triangle"},
	{ISOSURFACE_TOKEN, "isosurface"},
	{JITTER_TOKEN, "jitter"},
	{JULIA_TOKEN, "julia"},
	{JULIA_FRACTAL_TOKEN, "julia_fractal"},
	{JPEG_TOKEN, "jpeg"},
	{KEEP_TOKEN,"keep"},
	{LAMBERTAZIMUTHAL_TOKEN, "lambert_azimuthal"},
	{LAMBERTCYLINDRICAL_TOKEN, "lambert_cylindrical"},
	{LAMBDA_TOKEN, "lambda"},
	{LATHE_TOKEN, "lathe"},
	{LEFT_ANGLE_TOKEN, "<"},
	{LEFT_CURLY_TOKEN, "{"},
	{LEFT_PAREN_TOKEN, "("},
	{LEFT_SQUARE_TOKEN, "["},
	{LEOPARD_TOKEN, "leopard"},
	{LIGHT_GROUP_TOKEN, "light_group"},
	{LIGHT_SOURCE_TOKEN, "light_source"},
	{LINEAR_SPLINE_TOKEN, "linear_spline"},
	{LINEAR_SWEEP_TOKEN, "linear_sweep"},
	{LN_TOKEN, "ln"},
	{LOAD_FILE_TOKEN, "load_file"},
	{LOCAL_TOKEN, "local"},
	{LOCATION_TOKEN, "location"},
	{LOG_TOKEN, "log"},
	{LOOKS_LIKE_TOKEN, "looks_like"},
	{LOOK_AT_TOKEN, "look_at"},
	{LOW_ERROR_FACTOR_TOKEN, "low_error_factor" },
	{MACRO_ID_TOKEN, "macro identifier"},
	{MACRO_TOKEN, "macro"},
	{MAGNET_TOKEN, "magnet"},
	{MAJOR_RADIUS_TOKEN, "major_radius"},
	{MANDEL_TOKEN, "mandel"},
	{MAP_TYPE_TOKEN, "map_type"},
	{MARBLE_TOKEN, "marble"},
	{MASONRY_TOKEN, "masonry"},
	{MATERIAL_ID_TOKEN, "material identifier"},
	{MATERIAL_MAP_TOKEN, "material_map"},
	{MATERIAL_TOKEN, "material"},
	{MATRIX_TOKEN, "matrix"},
	{MAXIMAL_TOKEN, "maximal"},
	{MAX_EXTENT_TOKEN, "max_extent"},
	{MAX_GRADIENT_TOKEN, "max_gradient"},
	{MAX_INTERSECTIONS_TOKEN, "max_intersections"},
	{MAX_ITERATION_TOKEN, "max_iteration"},
	{MAX_SAMPLE_TOKEN, "max_sample"},
	{MAX_TOKEN, "max"},
	{MAX_TRACE_LEVEL_TOKEN, "max_trace_level"},
	{MAX_TRACE_TOKEN, "max_trace"},
	{MEDIA_ATTENUATION_TOKEN, "media_attenuation"},
	{MEDIA_ID_TOKEN, "media identifier"},
	{MEDIA_INTERACTION_TOKEN, "media_interaction"},
	{MEDIA_TOKEN, "media"},
	{MERGE_TOKEN, "merge"},
	{MERCATOR_TOKEN, "mercator"},
	{MESH_CAMERA_TOKEN, "mesh_camera"},
	{MESH2_TOKEN, "mesh2"},
	{MESH_TOKEN, "mesh"},
	{METALLIC_TOKEN, "metallic"},
	{METHOD_TOKEN, "method"},
	{METRIC_TOKEN, "metric" },
	{MILLERCYLINDRICAL_TOKEN, "miller_cylindrical"},
	{MINIMAL_TOKEN, "minimal"},
	{MAXIMUM_REUSE_TOKEN, "maximum_reuse" },
	{MINIMUM_REUSE_TOKEN, "minimum_reuse" },
	{MIN_EXTENT_TOKEN, "min_extent" },
	{MIN_TOKEN, "min"},
	{MM_PER_UNIT_TOKEN, "mm_per_unit"},
	{MOD_TOKEN, "mod"},
	{MODULATION_TOKEN, "modulation"},
	{MOLLWEIDE_TOKEN, "mollweide"},
	{MORTAR_TOKEN, "mortar"},
	{MOVE_TOKEN, "move"},
	{NATURAL_SPLINE_TOKEN, "natural_spline"},
	{NEAREST_COUNT_TOKEN, "nearest_count" },
	{NOISE_GENERATOR_TOKEN, "noise_generator"},
	{NORMAL_INDICES_TOKEN, "normal_indices"},
	{NORMAL_MAP_ID_TOKEN, "normal_map identifier"},
	{NORMAL_MAP_TOKEN, "normal_map"},
	{NORMAL_VECTORS_TOKEN, "normal_vectors"},
	{NO_BUMP_SCALE_TOKEN, "no_bump_scale"},
	{NO_IMAGE_TOKEN, "no_image"},
	{NO_RADIOSITY_TOKEN, "no_radiosity"},
	{NO_REFLECTION_TOKEN, "no_reflection"},
	{NO_SHADOW_TOKEN, "no_shadow"},
	{NO_TOKEN, "no"},
	{NOW_TOKEN, "now"},
	{NUMBER_OF_SIDES_TOKEN, "number_of_sides"},
	{NUMBER_OF_TILES_TOKEN, "number_of_tiles"},
	{NUMBER_OF_WAVES_TOKEN, "number_of_waves"},
	{OBJECT_ID_TOKEN, "object identifier"},
	{OBJECT_TOKEN, "object"},
	{OCTA_TOKEN, "octa"},
	{OCTAVES_TOKEN, "octaves"},
	{OFFSET_TOKEN, "offset"},
	{OFF_TOKEN, "off"},
	{OMEGA_TOKEN, "omega"},
	{OMNIMAX_TOKEN, "omnimax"},
	{ONCE_TOKEN, "once"},
	{ONION_TOKEN, "onion"},
	{ON_TOKEN, "on"},
	{OPEN_TOKEN, "open"},
	{ORIENTATION_TOKEN, "orientation"},
	{ORIENT_TOKEN,"orient"},
	{ORIGIN_TOKEN,"origin"},
	{ORIGINAL_TOKEN,"original"},
	{ORTHOGRAPHIC_TOKEN, "orthographic"},
	{OUTSIDE_TOKEN,"outside"},
	{OVUS_TOKEN, "ovus"},
	{PANORAMIC_TOKEN, "panoramic"},
	{PARALLAXE_TOKEN, "parallaxe"},
	{PARALLEL_TOKEN, "parallel"},
	{PARAMETER_ID_TOKEN, "parameter identifier"},
	{PARAMETRIC_TOKEN,"parametric"},
	{PASS_THROUGH_TOKEN, "pass_through"},
	{PATTERN_TOKEN, "pattern"},
	{PAVEMENT_TOKEN, "pavement"},
	{PERCENT_TOKEN, "%"},
	{PERIOD_TOKEN, ". (period)"},
	{PERSPECTIVE_TOKEN, "perspective"},
	{PETERS_TOKEN, "peters"},
	{PGM_TOKEN, "pgm"},
	{PHASE_TOKEN, "phase"},
	{PHONG_SIZE_TOKEN, "phong_size"},
	{PHONG_TOKEN, "phong"},
	{PHOTONS_TOKEN, "photons"},
	{PIGMENT_ID_TOKEN, "pigment identifier"},
	{PIGMENT_MAP_ID_TOKEN, "pigment_map identifier"},
	{PIGMENT_MAP_TOKEN, "pigment_map"},
	{PIGMENT_PATTERN_TOKEN,"pigment_pattern"},
	{PIGMENT_TOKEN, "pigment"},
	{PI_TOKEN, "pi"},
	{PLANAR_TOKEN, "planar"},
	{PLANE_TOKEN, "plane"},
	{PLANET_TOKEN, "planet"},
	{PLATECARREE_TOKEN, "plate_carree"},
	{PLUS_TOKEN, "+"},
	{PNG_TOKEN, "png"},
	{POINT_AT_TOKEN, "point_at"},
	{POLYGON_TOKEN, "polygon"},
	{POLY_TOKEN, "poly"},
	{POLYNOM_TOKEN, "polynomial"},
	{POLY_WAVE_TOKEN, "poly_wave"},
	{POT_TOKEN, "pot"},
	{POV_TOKEN, "pov"},
	{POW_TOKEN, "pow"},
	{PPM_TOKEN, "ppm"},
	{PRECISION_TOKEN, "precision"},
	{PRECOMPUTE_TOKEN,"precompute"},
	{PREMULTIPLIED_TOKEN,"premultiplied"},
	{PRETRACE_START_TOKEN, "pretrace_start"},
	{PRETRACE_END_TOKEN, "pretrace_end"},
	{PRISM_TOKEN, "prism"},
	{PROD_TOKEN, "prod"},
	{PROJECTED_THROUGH_TOKEN, "projected_through"},
	{PROPORTION_TOKEN,"proportion"},
	{PROXIMITY_TOKEN, "proximity"},
	{PWR_TOKEN, "pwr"},
	{QUADRATIC_SPLINE_TOKEN, "quadratic_spline"},
	{QUADRIC_TOKEN, "quadric"},
	{QUARTIC_TOKEN, "quartic"},
	{QUATERNION_TOKEN, "quaternion"},
	{QUESTION_TOKEN, "?"},
	{QUICK_COLOUR_TOKEN, "quick_color"},
	{QUICK_COLOUR_TOKEN, "quick_colour"},
	{QUILTED_TOKEN, "quilted"},
	{RADIAL_TOKEN, "radial"},
	{RADIANS_TOKEN, "radians"},
	{RADIOSITY_TOKEN, "radiosity" },
	{RADIUS_TOKEN, "radius"},
	{RAINBOW_ID_TOKEN, "rainbow identifier"},
	{RAINBOW_TOKEN, "rainbow"},
	{RAMP_WAVE_TOKEN, "ramp_wave"},
	{RAND_TOKEN, "rand"},
	{RANGE_TOKEN, "range"},
	{RATIO_TOKEN, "ratio"},
	{READ_TOKEN, "read"},
	{RECIPROCAL_TOKEN, "reciprocal" },
	{RECURSION_LIMIT_TOKEN, "recursion_limit" },
	{RED_TOKEN, "red"},
	{REFLECTION_EXPONENT_TOKEN, "reflection_exponent"},
	{REFLECTION_TOKEN, "reflection"},
	{REFRACTION_TOKEN, "refraction"},
	{REL_GE_TOKEN, ">="},
	{REL_LE_TOKEN, "<="},
	{REL_NE_TOKEN, "!="},
	{RENDER_TOKEN, "render"},
	{REPEAT_TOKEN, "repeat"},
	{RGBFT_TOKEN, "rgbft"},
	{RGBF_TOKEN, "rgbf"},
	{RGBT_TOKEN, "rgbt"},
	{RGB_TOKEN, "rgb"},
	{RIGHT_ANGLE_TOKEN, ">"},
	{RIGHT_CURLY_TOKEN, "}"},
	{RIGHT_PAREN_TOKEN, ")"},
	{RIGHT_SQUARE_TOKEN, "]"},
	{RIGHT_TOKEN, "right"},
	{RIPPLES_TOKEN, "ripples"},
	{ROLL_TOKEN,"roll"},
	{ROTATE_TOKEN, "rotate"},
	{ROUGHNESS_TOKEN, "roughness"},
	{SAMPLES_TOKEN, "samples"},
	{SAVE_FILE_TOKEN, "save_file"},
	{SCALE_TOKEN, "scale"},
	{SCALLOP_WAVE_TOKEN, "scallop_wave"},
	{SCATTERING_TOKEN, "scattering"},
	{SCREW_TOKEN,"screw"},
	{SEED_TOKEN, "seed"},
	{SELECT_TOKEN, "select"},
	{SEMI_COLON_TOKEN, ";"},
	{SINE_WAVE_TOKEN, "sine_wave"},
	{SINGLE_QUOTE_TOKEN, "'"},
	{SINH_TOKEN, "sinh"},
	{SIN_TOKEN, "sin"},
	{SINT16BE_TOKEN, "sint16be"},
	{SINT16LE_TOKEN, "sint16le"},
	{SINT32BE_TOKEN, "sint32be"},
	{SINT32LE_TOKEN, "sint32le"},
	{SINT8_TOKEN, "sint8"},
	{SIZE_TOKEN, "size" },
	{SKYSPHERE_ID_TOKEN, "sky_sphere identifier"},
	{SKYSPHERE_TOKEN, "sky_sphere"},
	{SKY_TOKEN, "sky"},
	{SLASH_TOKEN, "/"},
	{SLICE_TOKEN, "slice"},
	{SLOPE_MAP_ID_TOKEN, "slope_map identifier"},
	{SLOPE_MAP_TOKEN, "slope_map"},
	{SLOPE_TOKEN,"slope"},
	{SMOOTH_TOKEN, "smooth"},
	{SMOOTH_TRIANGLE_TOKEN, "smooth_triangle"},
	{SMYTH_CRASTER_TOKEN, "smyth_craster"},
	{SOLID_TOKEN, "solid" },
	{SOR_TOKEN, "sor"},
	{SOR_SPLINE_TOKEN, "sor_spline"},
	{SPACING_TOKEN, "spacing"},
	{SPECULAR_TOKEN, "specular"},
	{SPHERE_SWEEP_TOKEN, "sphere_sweep"},
	{SPHERE_TOKEN, "sphere"},
	{SPHERICAL_TOKEN, "spherical"},
	{SPIRAL1_TOKEN, "spiral1"},
	{SPIRAL2_TOKEN, "spiral2"},
	{SPLINE_ID_TOKEN, "spline identifier"},
	{SPLINE_TOKEN, "spline"},
	{SPLIT_UNION_TOKEN, "split_union"},
	{SPOTLIGHT_TOKEN, "spotlight"},
	{SPOTTED_TOKEN, "spotted"},
#if 0 // sred, sgreen and sblue tokens not enabled at present
	{SBLUE_TOKEN, "sblue"},
	{SGREEN_TOKEN, "sgreen"},
#endif
	{SQRT_TOKEN, "sqrt"},
	{SQR_TOKEN, "sqr"},
	{SQUARE_TOKEN, "square"},
#if 0 // sred, sgreen and sblue tokens not enabled at present
	{SRED_TOKEN, "sred"},
#endif
	{SRGB_TOKEN, "srgb"},
	{SRGBF_TOKEN, "srgbf"},
	{SRGBFT_TOKEN, "srgbft"},
	{SRGBT_TOKEN, "srgbt"},
	{STAR_TOKEN, "*"},
	{STATISTICS_TOKEN, "statistics"},
	{STEPS_TOKEN,"steps"},
	{STEREO_TOKEN,"stereo"},
	{STL_SAVE_TOKEN,"stl_save"},
	{STRCMP_TOKEN, "strcmp"},
	{STRENGTH_TOKEN, "strength"},
	{STRING_ID_TOKEN, "string identifier"},
	{STRING_LITERAL_TOKEN, "string literal"},
	{STRLEN_TOKEN, "strlen"},
	{STRLWR_TOKEN, "strlwr"},
	{STRUPR_TOKEN, "strupr"},
	{STR_TOKEN, "str"},
	{STURM_TOKEN, "sturm"},
	{SUBSURFACE_TOKEN, "subsurface"},
	{SUBSTR_TOKEN, "substr"},
	{SUM_TOKEN, "sum"},
	{SUPERELLIPSOID_TOKEN, "superellipsoid"},
	{SWITCH_TOKEN, "switch"},
	{SYS_TOKEN, "sys"},
	{TANH_TOKEN, "tanh"},
	{TAN_TOKEN, "tan"},
	{TARGET_TOKEN, "target"},
	{TCB_SPLINE_TOKEN, "tcb_spline"}, 
	{TESSELATE_TOKEN,"tesselate"},
	{TESSEL_TOKEN,"tessel"},
	{TENSION_TOKEN, "tension"},
	{TEMPORARY_MACRO_ID_TOKEN, "unfinished macro declaration"},
	{TETRA_TOKEN, "tetra"},
	{TEXTURE_ID_TOKEN, "texture identifier"},
	{TEXTURE_LIST_TOKEN, "texture_list"},
	{TEXTURE_MAP_ID_TOKEN, "texture_map identifier"},
	{TEXTURE_MAP_TOKEN, "texture_map"},
	{TEXTURE_TOKEN, "texture"},
	{TEXT_TOKEN, "text"},
	{TGA_TOKEN, "tga"},
	{THICKNESS_TOKEN, "thickness"},
	{THRESHOLD_TOKEN, "threshold"},
	{TIFF_TOKEN, "tiff"},
	{TIGHTNESS_TOKEN, "tightness"},
	{TILDE_TOKEN, "~"},
	{TILE2_TOKEN, "tile2"},
	{TILES_TOKEN, "tiles"},
	{TILING_TOKEN, "tiling"},
	{TNORMAL_ID_TOKEN, "normal identifier"},
	{TNORMAL_TOKEN, "normal"},
	{TOLERANCE_TOKEN, "tolerance"},
	{TOROIDAL_TOKEN, "toroidal"},
	{TORUS_TOKEN, "torus"},
	{TRACE_TOKEN, "trace"},
	{TRANSFORM_ID_TOKEN, "transform identifier"},
	{TRANSFORM_TOKEN, "transform"},
	{TRANSLATE_TOKEN, "translate"},
	{TRANSLUCENCY_TOKEN, "translucency"},
	{TRANSMIT_TOKEN, "transmit"},
	{TRIANGLE_TOKEN, "triangle"},
	{TRIANGLE_WAVE_TOKEN, "triangle_wave"},
	{TRIANGULAR_TOKEN, "triangular"},
	{TRUE_TOKEN, "true"},
	{TTF_TOKEN, "ttf"},
	{TURBULENCE_TOKEN, "turbulence"},
	{TURB_DEPTH_TOKEN, "turb_depth"},
	{TYPE_TOKEN, "type"},
	{T_TOKEN, "t"},
	{UINT16BE_TOKEN, "uint16be"},
	{UINT16LE_TOKEN, "uint16le"},
	{UINT8_TOKEN, "uint8"},
	{ULTRA_WIDE_ANGLE_TOKEN, "ultra_wide_angle"},
	{UNDEF_TOKEN, "undef"},
	{UNION_TOKEN, "union"},
	{UP_TOKEN, "up"},
	{USE_ALPHA_TOKEN, "use_alpha"},
	{USE_COLOUR_TOKEN, "use_color"},
	{USE_COLOUR_TOKEN, "use_colour"},
	{USE_INDEX_TOKEN, "use_index"},
	{UTF8_TOKEN, "utf8"},
	{UV_ID_TOKEN, "uv vector identifier"},
	{UV_INDICES_TOKEN, "uv_indices"},
	{UV_MAPPING_TOKEN, "uv_mapping"},
	{UV_VECTORS_TOKEN, "uv_vectors"},
	{U_STEPS_TOKEN, "u_steps"},
	{U_TOKEN, "u"},
	{VAL_TOKEN, "val"},
	{VARIANCE_TOKEN, "variance"},
	{VAXIS_ROTATE_TOKEN, "vaxis_rotate"},
	{VCROSS_TOKEN, "vcross"},
	{VDOT_TOKEN, "vdot"},
	{VECTFUNCT_ID_TOKEN, "vector function identifier"},
	{VECTOR_4D_ID_TOKEN, "4d-vector identifier"},
	{VECTOR_FUNCT_TOKEN, "vector function"},
	{VECTOR_ID_TOKEN, "vector identifier"},
	{VERSION_TOKEN, "version"},
	{VERTEX_VECTORS_TOKEN, "vertex_vectors"},
	{VLENGTH_TOKEN, "vlength"},
	{VNORMALIZE_TOKEN, "vnormalize"},
	{VROTATE_TOKEN, "vrotate"},
	{VSTR_TOKEN, "vstr"},
	{VTURBULENCE_TOKEN, "vturbulence"},
	{V_STEPS_TOKEN, "v_steps"},
	{V_TOKEN, "v"},
	{VAN_DER_GRINTEN_TOKEN, "van_der_grinten"},
	{VORONOI_TOKEN, "voronoi"},
	{WARNING_TOKEN, "warning"},
	{WARP_TOKEN, "warp"},
	{WATER_LEVEL_TOKEN, "water_level"},
	{WAVES_TOKEN, "waves"},
	{WHILE_TOKEN, "while"},
	{WIDTH_TOKEN, "width"},
	{WITH_TOKEN, "with"},
	{WOOD_TOKEN, "wood"},
	{WRINKLES_TOKEN, "wrinkles"},
	{WRITE_TOKEN, "write"},
	{X_TOKEN, "x"},
	{XYL_TOKEN, "xyl"},
	{XYV_TOKEN, "xyv"},
	{XYZ_TOKEN, "xyz"},
	{YES_TOKEN, "yes"},
	{Y_TOKEN, "y"},
	{Z_TOKEN, "z"}
};

} // end of pov namespace
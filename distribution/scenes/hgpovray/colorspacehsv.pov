#version 3.7;
global_settings { assumed_gamma 1.0 }


#include "colors.inc"

camera {
   location  <0, 0, -5>
   right     x*image_width/image_height
   direction <0, 0, 1.7>
   look_at   <0, 0, 0>
}

background { color rgb<1,1,1>*0.02 } 

light_source { <-15, 30, -25> color red 1/2 green 1/2 blue 1/2 }
light_source { < 15, 30, -25> color red 1/2 green 1/2 blue 1/2 }
#declare Finish=

   finish {
      emission 0.5
      diffuse 0.4
      phong 1
   };
blob {
   threshold 0.6
   sphere { <0.75, 0, 0>,1,1 pigment { color rgb< 0.7,0.2,0.05> } finish { Finish } }
   sphere { <-0.375, 0.64952, 0>,1,1 pigment { color rgb< 0.05, 0.2,1> } finish { Finish } }
   sphere { <-0.375, -0.64952, 0>,1,1 pigment { color rgb< 0.2,0.7, 0.05> } finish { Finish } }
   sphere { <-1.5, 0, 0>,1,1 pigment { color rgb< 1,1,0> } finish { Finish } }
   sphere { <-1.5, 1.29904, 0>,1,1 pigment { color rgb< 1,0,0> } finish { Finish } }
   sphere { <0.75, -1.29904, 0>,1,1 pigment { color rgb< 0.5,0,0> } finish { Finish } }
   sphere { <0.75, 1.29904, 0>,1,1 pigment { color rgb< 1,1,1>/7 } finish { Finish } }
  color_space hsv

   rotate 30*y
}

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

light_source { <-15, 30, -25> color red 1 green 1 blue 1 }
light_source { < 15, 30, -25> color red 1 green 1 blue 1 }

#declare colo1=texture { pigment {color rgb<1.,1.,0>}};
#declare colo2=texture { pigment {color rgb<0.,1.,2/3>}};
#declare colo3=texture { pigment {color rgb<1.,0.,1/3>}};

mesh { triangle {x+y, x, y texture_list { colo1, colo2, colo3 }  } color_space xyv translate 0.2*(x+y) }
mesh { triangle {x+y, x, y texture_list { colo1, colo2, colo3 }  } color_space xyl  translate 0.2*(x+y) scale <-1,1,1>  }
mesh { triangle {x+y, x, y texture_list { colo1, colo2, colo3 }  } color_space hsv  translate 0.2*(x+y) rotate 180*z }
mesh { triangle {x+y, x, y texture_list { colo1, colo2, colo3 }  } color_space hsl  translate 0.2*(x+y) scale <1,-1,1> }

mesh { 
triangle {0, x, y texture_list { colo1, colo2, colo3 } }
triangle {0, x, -y texture_list { colo1, colo2, colo3 } }
triangle {0, -x, -y texture_list { colo1, colo2, colo3 } }
triangle {0, -x, y texture_list { colo1, colo2, colo3 } }
 color_space pov 
}

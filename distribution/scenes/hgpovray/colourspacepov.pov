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

#declare colo1=texture { pigment {color rgb<1.,0.,0>}};
#declare colo2=texture { pigment {color rgb<0.,1.,1/3>}};
#declare colo3=texture { pigment {color rgb<0.,1.,1>}};
#declare colo4=texture { pigment {color rgb<1.,1.,1/3>}};
#declare colo5=texture { pigment {color rgb<1.,1./3,1>}};
#declare colo6=texture { pigment {color rgb<1./3.,1.,0>}};

#declare num=6;
polygon { (num+1),
#local i=0;
#while(i<num+1)
	< cos(i*2*pi/num),sin(i*2*pi/num),0>
#local i=i+1;
#end

texture_list { colo1, colo2, colo3, colo4, colo5, colo6 } 
strength 2.900

color_space pov
}

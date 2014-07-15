#version 3.7;
global_settings { assumed_gamma 1.0 }

camera { location -560*z
direction z
up y
right image_width*x/image_height
angle 5
}

#include "colors.inc"

intermerge{
#for(i,0,359.99,60)
#local pos= vrotate(6*x,i*z);
cylinder { pos-z,pos+31*z,10 texture { pigment { color CH2RGB(i) filter 0.45 } } }
#end
range { <2,4> }
clipped_by {cylinder { 30*z,0, 20  } }
}
box { 30*z-20*x-20*y, 30*z+20*x+20*y texture { pigment { color White }}}

light_source { 10*<0,0,-20>, 0.9 }
light_source { 10*<-5,10,-50>, 0.9 }
light_source { 10*<-10,10,-50>, 0.9 }
light_source { 10*<5,10,-50>, 0.9 }
light_source { 10*<10,10,-50>, 0.9 }

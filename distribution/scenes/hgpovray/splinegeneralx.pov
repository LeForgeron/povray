#version 3.7;
global_settings{assumed_gamma 1.0}

#include "colors.inc"

background { White }
camera { 
    orthographic  
    right x
    up y*image_height/image_width
    direction z 
    location -100*z-0.07*x
}
#local HFactor=5; 
#macro Spline_Entries()
    0 <0,0*HFactor> 
	1 <1/2,1*HFactor>
    2 <1,1*HFactor> 
    3 <2,-0.5*HFactor> 
    4 <3,-.5*HFactor> 
    5 <4,0.2*HFactor> 
    6 <6,0.2*HFactor> 
    7 <6,.7*HFactor> 
    8 <4,1*HFactor> 
    9 <7,1*HFactor> 
    10 <7,0.5*HFactor> 
	11 <7.2,0.5*HFactor>
	12 <7.2,1*HFactor>
	13 <7.4,1*HFactor>
	14 <7.4,0.5*HFactor>
	15 <7.6,0.5*HFactor>
	16 <7.6,1*HFactor>
	17 <7.8,1*HFactor>
	18 <7.8,0.5*HFactor>
	19 <8.0,0.5*HFactor>
	20 <8.0,1*HFactor>
	21 <8.2,1*HFactor>
	22 <8.2,0*HFactor>
	23 <8.4,0*HFactor>
	24 <8.4,1*HFactor>
	25 <8.6,1*HFactor>
	26 <8.6,0*HFactor>
	27 <8.8,0*HFactor>
	28 <8.8,1*HFactor>
	29 <9,1*HFactor>
	30 <9,0*HFactor>
    31 <10,-1*HFactor>
    32 <10,0*HFactor>
    33 <10.5,0*HFactor>
    34 <11.5,0*HFactor>
	35 <11,1*HFactor>
	36 <12,1*HFactor>
	37 <12,0*HFactor>
	38 <12.25,-1*HFactor>
	39 <12.5,0*HFactor>
	40 <12.75,1*HFactor>
	41 <13,0*HFactor>
	42 <13.25,-1*HFactor>
	43 <13.5,0*HFactor>
	44 <14,0*HFactor>
	45 <15,0*HFactor>
	46 <15,1*HFactor>
	47 <14,1*HFactor>
	48 <14,0*HFactor>
	49 <15,0*HFactor>
	50 <16,0*HFactor>
#end

#declare Spline_Base=spline{ Spline_Entries() };

#declare Control_Point_Pigment=pigment{color Red};
#declare Control_Point_Radius=0.05*2;

#declare Spline_Pigment=pigment{color Black};
#declare Spline_Radius=0.04*2;
#declare Spline_Accuracy=0.01;

#declare Position = -1/2;

#macro Draw_Spline(Spline)
    union{
			  cylinder { <0,0,0>,<16,0,0>,Spline_Radius/2 pigment { color  Grey}}
        #local Counter = 0;
        #while (Counter < dimension_size(Spline))
            #local Center = Spline_Base(Counter);
            sphere{ 
                <Center.x,Center.y,0> 
                Control_Point_Radius 
                pigment{ Control_Point_Pigment }
            }
            #if (Counter>0)
            cylinder{ 
                <Spline_Base(Counter-1).x,Spline_Base(Counter-1).y,0> 
                <Center.x,Center.y,0> 
                Spline_Radius/2
                pigment{ color Green }
            }
            #end
            #local Center = Spline(Counter);
            #local C = Spline_Accuracy;
            #while (C <= 1)
                #local New_Center = Spline(Counter + C);
                    sphere{ 
                        <New_Center.x,New_Center.y,0> 
                        Spline_Radius 
                        pigment{ Spline_Pigment }
                    }
                #local Center = New_Center;
                #local C=C+Spline_Accuracy;
            #end
            #local Counter = Counter + 1;
        #end
        scale 1/18
        translate -x/2-y/40
        #declare Position = Position + 1;
    }
#end


#default{finish{ambient 1 diffuse 0}}

Draw_Spline(spline{general_x_spline freedom_degree -1/2 Spline_Entries()})

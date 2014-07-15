#version 3.7;
global_settings{ charset utf8 assumed_gamma 1.0 }
#default{ finish { emission 1 diffuse 0 } }
#declare Teo="Li Europan lingues es membres del sam familie. Lor separat existentie es un myth. Por scientie, musica, sport etc, litot Europa usa li sam vocabular. Li lingues differe solmen in li grammatica, li pronunciation e li plu commun vocabules. Omnicos directe al desirabilite de un nov lingua franca: On refusa continuar payar custosi traductores.\n At solmen va esser necessi far uniform grammatica, pronunciation e plu commun paroles. Ma quande lingues coalesce, li grammatica del resultant lingue es plu simplic e regulari quam ti del coalescent lingues. Li nov lingua franca va esser plu simplic e regulari quam li existent Europanlingues. It va esser tam simplic quam Occidental in fact, it va esser Occidental. A un Angleso it va semblar un simplificat Angles, quam un skeptic Cambridge amico dit me que Occidental es. ";

#declare mar=0.181;
#declare k=30;
#declare heig=8.5;
#for(i,0,1,1)
#for(j,-2,2,2)
galley{ internal 2, Teo, <0.01,1.5,i>, <k, -1, j>
translate (heig*j+8)*y-(k+3)*i*x no_shadow
}
cylinder { -(heig*2-1)*y,2*y,mar texture { pigment { color green 1 } } translate (j)*x-(k+3)*i*x +6*z+(heig*j+8)*y }
#end

cylinder { -40*y,40*y,mar texture { pigment { color rgbf <1,1,0,0.5> } } translate -(k+3)*i*x +5*z }
cylinder { -40*y,40*y,mar texture { pigment { color rgb <0,1,1> } } translate +k*x-(k+3)*i*x +5*z }
#end

camera { orthographic
location <0,2,-30>
up y
right image_width/image_height*x
direction z
look_at 2*y
angle 100
}

#declare IS=texture { pigment { color srgb <253,248,242>/255  } } ;

plane { z,0 texture { IS } translate 10*z }

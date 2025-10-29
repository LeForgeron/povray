#include "colors.inc"

global_settings { assumed_gamma 1.0 }

camera {
    location <10, 8, 12>
    look_at <0, 0, 0>
    angle 60
}

light_source {
    <20, 20, 20>
    color White
}

background { color rgb <0.2, 0.2, 0.3> }

// Define atom properties
#declare AtomRadius = 0.4;
#declare LatticeConstant = 2.0;  // Distance between atoms
#declare CrystalSize = 3;        // Number of atoms in each direction

// Atom material
#declare AtomMaterial = material {
    texture {
        pigment { color rgb <0.8, 0.2, 0.2> }  // Red color
        finish {
            ambient 0.2
            diffuse 0.6
            specular 0.8
            roughness 0.1
        }
    }
}

// Create simple cubic lattice
#declare i = -CrystalSize;
#while (i <= CrystalSize)
    #declare j = -CrystalSize;
    #while (j <= CrystalSize)
        #declare k = -CrystalSize;
        #while (k <= CrystalSize)
            sphere {
                <i * LatticeConstant, j * LatticeConstant, k * LatticeConstant>, AtomRadius
                material { AtomMaterial }
            }
            #declare k = k + 1;
        #end
        #declare j = j + 1;
    #end
    #declare i = i + 1;
#end

// Optional: Add bonds between nearest neighbors
#declare BondRadius = 0.08;
#declare BondMaterial = material {
    texture {
        pigment { color rgb <0.7, 0.7, 0.7> }  // Gray color
        finish {
            ambient 0.1
            diffuse 0.5
        }
    }
}

// Create bonds (uncomment if you want bonds)
/*
#declare i = -CrystalSize;
#while (i < CrystalSize)
    #declare j = -CrystalSize;
    #while (j <= CrystalSize)
        #declare k = -CrystalSize;
        #while (k <= CrystalSize)
            // X-direction bonds
            cylinder {
                <i * LatticeConstant, j * LatticeConstant, k * LatticeConstant>,
                <(i+1) * LatticeConstant, j * LatticeConstant, k * LatticeConstant>,
                BondRadius
                material { BondMaterial }
            }
            // Y-direction bonds
            cylinder {
                <i * LatticeConstant, j * LatticeConstant, k * LatticeConstant>,
                <i * LatticeConstant, (j+1) * LatticeConstant, k * LatticeConstant>,
                BondRadius
                material { BondMaterial }
            }
            // Z-direction bonds
            cylinder {
                <i * LatticeConstant, j * LatticeConstant, k * LatticeConstant>,
                <i * LatticeConstant, j * LatticeConstant, (k+1) * LatticeConstant>,
                BondRadius
                material { BondMaterial }
            }
            #declare k = k + 1;
        #end
        #declare j = j + 1;
    #end
    #declare i = i + 1;
#end
*/

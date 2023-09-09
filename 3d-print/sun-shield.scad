/*-------------------------------------------------------------------
 title: WX station termistor sun shield
 author: OK1HRA 
 license: Creative Commons BY-SA
 URL: http://remoteqth.com/3d-wx-station.php
 revision: 0.1
 format: OpenSCAD
-------------------------------------------------------------------
 HowTo:
 After open change inputs parameters and press F5
 .STL export press F6 and menu /Design/Export as STL...
-------------------------------------------------------------------
 Changelog:
      2023-09 - minor bugfix
      2021-01 - initial relase
------------------------------------------------------------------- */

MOUNTDIA     =    42;    // mounting pipe diameter in mm
Part                     =       0;    // select part 1-5, 0=preview All

//-------------------------------------------------------------------

// don't edit this variables
OutsideDiameter = 150;
InsideDiameter     = OutsideDiameter-30;
VerticalEdge  = 20;
WallThickness = 2;
VerticalDistance = 13;
M3diaFrom = 3.0;
M3diaTo = 2.7;

//-------------------------------------------------------------------



// 4stl
    if(Part==1){
        // top - dependency to mountdia
        translate([0,0,-13*2]) baseTop(OutsideDiameter, InsideDiameter, VerticalEdge, WallThickness, 10, VerticalDistance, MOUNTDIA,0);
    }
    if(Part==2){
        // under top - dependency to mountdia
        translate([0,0,-13]) baseTop(OutsideDiameter, InsideDiameter, VerticalEdge, WallThickness, 10, VerticalDistance, MOUNTDIA,60);
    }
    if(Part==3){
        // moutpoint part - dependency to mountdia
        base(OutsideDiameter, InsideDiameter, VerticalEdge, WallThickness*2, 10, VerticalDistance, MOUNTDIA);
    }
    if(Part==4){
        // bottom sunshield
        translate([0,0,13]) Shield(OutsideDiameter, InsideDiameter, VerticalEdge, WallThickness, 10, VerticalDistance);
    }
    if(Part==5){
        // bottom protect
         translate([0,0,65]) rotate([180,0,0]) bottom(VerticalEdge);
    }
    if(Part==0){
// preview complete
        difference(){
            color([1,1,1]) union(){
                // top
                translate([0,0,-13*2]) baseTop(OutsideDiameter, InsideDiameter, VerticalEdge, WallThickness, 10, VerticalDistance, MOUNTDIA,0);
                // under top
                translate([0,0,-13]) baseTop(OutsideDiameter, InsideDiameter, VerticalEdge, WallThickness, 10, VerticalDistance, MOUNTDIA,60);
                // moutpoint part
                base(OutsideDiameter, InsideDiameter, VerticalEdge, WallThickness*2, 10, VerticalDistance, MOUNTDIA);
#                cylinder(h=50,d=6,center=false,$fn=30);
                // bottom sunshield
                 for(ZZZ = [13 : 13 : 60]){
                    translate([0,0,ZZZ]) Shield(OutsideDiameter, InsideDiameter, VerticalEdge, WallThickness, 10, VerticalDistance);
                }
                // bottom protect
                 translate([0,0,65])  bottom(VerticalEdge);
            }
         translate([-125,0,-100]) cube([250,200,400]);
        }
    }
// not use
// translate([20,0,43-15])  dome(205, 2, MOUNTDIA);


module bottom(VerticalEdge){
    difference(){
    cylinder(h=VerticalEdge-10, d1=116, d2=95+2, $fn=300, center=false);
    translate([0,0,-1])cylinder(h=9, d=95, $fn=300, center=false);
          // M3 hole
             for(R = [-60: 120 : 300]){
                rotate([0,0,R]) translate([120/2-10/2-2,0,-1]) cylinder(h=12, d=3.2, $fn=30, center=false);
                rotate([0,0,R]) translate([120/2-10/2-2,0,+2]) cylinder(h=12, d=6.5, $fn=30, center=false);
             }
    }
 }

module baseTop(DIAb, DIAs, ZZ, WALL, DIAi, SPACE, MOUNTDIA, ROTATE){
    difference(){
        union(){
            difference(){
                // outside
                union(){
                    hull(){
                        cylinder(h=ZZ, d1=DIAs, d2=DIAb, $fn=400, center=false);
//                        translate([DIAb/2,0,ZZ/2]) cube([1,MOUNTDIA+10,ZZ], center=true);
                        hull(){
                            translate([DIAb/2,0,ZZ/2]) cube([1,MOUNTDIA+10,ZZ], center=true);
                            translate([DIAb/2,0,ZZ-1]) cube([MOUNTDIA/3+10,MOUNTDIA+10,1], center=true);
                        }
                    }
                }
                hull(){
                    translate([0,0,WALL]) cylinder(h=ZZ-WALL+0.01, d1=DIAs-WALL*2, d2=DIAb-WALL*2, $fn=250, center=false);
                        hull(){
                            translate([DIAb/2-WALL,0,ZZ/2+WALL]) cube([1,MOUNTDIA+10,ZZ], center=true);
                            translate([DIAb/2-WALL/2,0,ZZ-1+WALL]) cube([MOUNTDIA/3+10,MOUNTDIA+10,1], center=true);
                        }

                }
                // - MOUNTDIA
                translate([DIAb/2+MOUNTDIA/2+10,0,0]) cylinder(h=ZZ+MOUNTDIA*1.5, d=MOUNTDIA+4, $fn=50, center=false);                
            }
            // IN
            if(ROTATE !=0){
                translate([0,0,WALL]) cylinder(h=SPACE-3, d=DIAs-DIAi*2.5, $fn=250, center=false);
            }
        }
        // -IN
        if(ROTATE !=0){
            translate([0,0,WALL]) cylinder(h=SPACE-3+0.01, d=DIAs-DIAi*2.5-2*WALL, $fn=250, center=false);
             for(R = [0 : 8 : 175]){
                rotate([0,0,R]) translate([0,0,7]) rotate([0,90,0]) cylinder(h=DIAs, d=5, center=true, $fn=30);
            }
        }
          // window
//         for(R = [-15 : 30 : 315]){
//            rotate([0,0,R]) translate([DIAs/2-DIAi/2-2,0,-1]) cylinder(h=WALL+2, d=6, $fn=30, center=false);
//         }
          // M3 hole
        if(ROTATE !=0){
             for(R = [-60+ROTATE : 120 : 300+ROTATE]){
                rotate([0,0,R]) translate([DIAs/2-DIAi/2-2,0,-1]) cylinder(h=WALL+2, d=3.2, $fn=30, center=false);
             }
         }
    }
    // spacers
     for(R = [0+ROTATE : 120 : 360+ROTATE]){
        rotate([0,0,R]) translate([DIAs/2-DIAi/2-2,0,0]) 
            difference(){
                cylinder(h=SPACE, d1=DIAi, d2=6, $fn=30, center=false);
                cylinder(h=SPACE+0.1, d=2.7, $fn=30, center=false);
                translate([0,0,SPACE-8.9])cylinder(h=9, d1=M3diaTo, d2=M3diaFrom, $fn=30, center=false);
            }
     }
}



module dome(DIA, WALL, MOUNTDIA){
         difference(){
            sphere(d=DIA, $fn=300); 
            difference(){ 
                sphere(d=DIA-2*WALL, $fn=50); 
                translate([0,0,-DIA*0.73+WALL*2]) cube([DIA+2,DIA+2,DIA], center=true);
            }
            translate([0,0,DIA/2]) cube([DIA*2,DIA+2,DIA], center=true);
            translate([0,0,-DIA*0.73]) cube([DIA+2,DIA+2,DIA], center=true);
//            translate([150/2+MOUNTDIA/3,0,-15]) cube([MOUNTDIA/3+10, MOUNTDIA+10+4, MOUNTDIA], center=true);
        }
 }


module base(DIAb, DIAs, ZZ, WALL, DIAi, SPACE, MOUNTDIA){
    difference(){
        union(){
            difference(){
                // outside
                union(){
                    hull(){
                        cylinder(h=ZZ, d1=DIAs, d2=DIAb, $fn=400, center=false);
                        translate([DIAb/2,0,ZZ/2]) cube([1,MOUNTDIA+10,ZZ], center=true);
                    }
                    hull(){
                        translate([DIAb/2+1,0,ZZ/2]) cube([1,MOUNTDIA+10,ZZ], center=true);
                        translate([DIAb/2+MOUNTDIA/3,0,MOUNTDIA]) cube([MOUNTDIA/3+10,MOUNTDIA+10,MOUNTDIA], center=true);
                    }
                }
                translate([0,0,WALL]) cylinder(h=ZZ-WALL+0.01, d1=DIAs-WALL*2, d2=DIAb-WALL*2, $fn=250, center=false);
                // - MOUNTDIA
                translate([DIAb/2+MOUNTDIA/2+10,0,0]) cylinder(h=ZZ+MOUNTDIA*1.5, d=MOUNTDIA, $fn=200, center=false);                
               translate([DIAb/2+MOUNTDIA/2+10,0, MOUNTDIA/2+4]) Binder(MOUNTDIA);
               translate([DIAb/2+MOUNTDIA/2+10,0,MOUNTDIA*1.5-6]) Binder(MOUNTDIA);
                rotate([0,0,22]) translate([DIAb/2+WALL,0,ZZ*0.7]) Binder(10);
            }
            // IN
            cylinder(h=ZZ, d1=DIAs-DIAi*2, d2=DIAs-DIAi*2-(DIAb-DIAs)/2, $fn=250, center=false);
        }
        // -IN
        translate([0,0,-0.01]) cylinder(h=ZZ+0.02, d1=DIAs-DIAi*2-WALL, d2=DIAs-DIAi*2-(DIAb-DIAs)/2-WALL, $fn=250, center=false);
          // window
         for(RR = [0 : 60 : 360]){
            for(R = [12 : 7 : 48]){
                rotate([0,0,R+RR]) translate([DIAs/2-DIAi/2-WALL/2,0,-1]) cylinder(h=WALL+2, d=5, $fn=30, center=false);
             }
         }
          // M3 hole
         for(R = [-60 : 120 : 300]){
            rotate([0,0,R]) translate([DIAs/2-DIAi/2-2,0,-1]) cylinder(h=WALL+2, d=3.2, $fn=30, center=false);
         }
    }
    // spacers
     for(R = [0 : 120 : 360]){
        rotate([0,0,R]) translate([DIAs/2-DIAi/2-2,0,0]) 
            difference(){
                cylinder(h=SPACE, d1=DIAi, d2=6, $fn=30, center=false);
                cylinder(h=SPACE+0.1, d=2.7, $fn=30, center=false);
                translate([0,0,SPACE-8.9])cylinder(h=9, d1=M3diaTo, d2=M3diaFrom, $fn=30, center=false);
            }
     }
     // center cross
     difference(){
         union(){
            translate([0,0,WALL/2]) cube([15,DIAs-DIAi*2-WALL,WALL], center=true);
            translate([0,0,WALL/2]) cube([DIAs-DIAi*2-WALL,15,WALL], center=true);
             cylinder(h=WALL+10, d1=15, d2=20, $fn=30, center=false);
         }
         translate([0,0,-1]) cylinder(h=ZZ+2, d=6.7, $fn=30, center=false);
         translate([-10,-2,WALL]) cube([20,20,ZZ]);
     }
}



module Shield(DIAb, DIAs, ZZ, WALL, DIAi, SPACE){
    difference(){
        union(){
            difference(){
                cylinder(h=ZZ, d1=DIAs, d2=DIAb, $fn=400, center=false);
                translate([0,0,WALL]) cylinder(h=ZZ-WALL+0.01, d1=DIAs-WALL*2, d2=DIAb-WALL*2, $fn=250, center=false);
            }
            // IN
            cylinder(h=ZZ, d1=DIAs-DIAi*2, d2=DIAs-DIAi*2-(DIAb-DIAs)/2, $fn=250, center=false);
        }
        // -IN
        translate([0,0,-0.01]) cylinder(h=ZZ+0.02, d1=DIAs-DIAi*2-WALL*2, d2=DIAs-DIAi*2-(DIAb-DIAs)/2-WALL*2, $fn=250, center=false);
          // window
         for(RR = [0 : 60 : 360]){
            for(R = [12 : 7 : 48]){
                rotate([0,0,R+RR]) translate([DIAs/2-DIAi/2-WALL/2,0,-1]) cylinder(h=WALL+2, d=5, $fn=30, center=false);
             }
         }
          // M3 hole
         for(R = [-60 : 120 : 300]){
            rotate([0,0,R]) translate([DIAs/2-DIAi/2-WALL,0,-1]) cylinder(h=WALL+2, d=3.2, $fn=30, center=false);
         }
    }
    // spacers
     for(R = [0 : 120 : 360]){
        rotate([0,0,R]) translate([DIAs/2-DIAi/2-WALL,0,0]) 
            difference(){
                cylinder(h=SPACE, d1=DIAi, d2=6, $fn=30, center=false);
                cylinder(h=SPACE+0.1, d=2.7, $fn=30, center=false);
                translate([0,0,SPACE-8.9])cylinder(h=9, d1=M3diaTo, d2=M3diaFrom, $fn=30, center=false);
            }
     }
     
}

module Binder(DIA){
    difference(){
        cylinder(h=6, d=DIA+6+6, $fn=100, center=true);
        cylinder(h=8, d=DIA+6, $fn=100, center=true);
    }
}


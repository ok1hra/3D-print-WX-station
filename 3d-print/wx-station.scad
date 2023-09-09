/*-------------------------------------------------------------------
 title: WX station
 author: OK1HRA 
 license: Creative Commons BY-SA
 URL: http://remoteqth.com/3d-wx-station.php
 format: OpenSCAD
-------------------------------------------------------------------
 HowTo:
         After open change inputs parameters and press F5
         .STL export press F6 and menu /Design/Export as STL...
-------------------------------------------------------------------
 Changelog:
        2022-09 - massive redesign to PCB rev 0.7
        2020-04 - add door and binder mount for external termistor wire
        2020-04 - initial relase
-------------------------------------------------------------------
        Legend
        BLUE     - PCB electronic
        RED      - neodym magnets
        GREEN    - bearings
//------------------------------------------------------------------- */
        Part             = -8;    // select part 1-18, -x=all, 17=variant of inner
                                   // ./render.sh bash script generate all parts
        BearingInTuning  = +20;    // +- smooth tuning bearing inner diameter in um
        MountDia         =  42;    // mm tube size for mounting
        MountDiaFn       = 100;    // mount tube polygon, 4 for square
        CUT              =   0;    // 0/1 hlaf cut view enable
        Strong           =   0;    // 1 enable strongest (part 6 only)
//-------------------------------------------------------------------

// PARAMETERS
MagnetX=2.2;
MagnetY=6.2;
MagnetZ=12;

RODshift=3.0;

PCBshift=2.7;
PCBwidth=14;

tloustka = 2;
height = 70;
space = 0.2;
M3 = 3.2;
diameter = 70;

difference(){
    Build2(Part);
    // -cut
    if (CUT>0&&Part<0) {
//    rotate([0,0,-90])
        translate([-70-5+5,0,100]) cube([140, 200, 400], center=true);
    }
}


module Build2(Part){
    // variant of inner
    if(Part==17){
        for(i= [-30 : 10 : 30]){
            translate([i*2,25,0]) In(40, 0.4, 1.6, i);    // ZZ, MINUS    // 10
            translate([i*2,0,0]) In(40, 0.4, 0, i);    // ZZ, MINUS    // 07
        }
    }    

    // Bearing press
    if(Part==16){
//        color([0,1,0])  translate([0, 0, 8.5+1]) rotate([0,90,0]) import("61802.stl", convexity = 5);
        difference(){
            union(){
                translate([0,0,6]) cylinder(h=4, d1=15, d2=14.5, center=false, $fn=200);
                translate([0,0,5]) cylinder(h=1.01, d=23.6, center=false, $fn=160);
                translate([0,0,0]) cylinder(h=5.01, d2=23.6, d1=15, center=false, $fn=160);
            }
            translate([0,0,-1]) cylinder(h=30, d=6.2, center=false, $fn=60);
            translate([0,0,-1]) cylinder(h=4, r=10/sin(60)/2, center=false, $fn=6);

        }

        difference(){
            union(){
                translate([30,0,6]) cylinder(h=2, d1=19, d2=18, center=false, $fn=200);
                translate([30,0,5]) cylinder(h=1.01, d=28, center=false, $fn=160);
                translate([30,0,0]) cylinder(h=5.01, d2=28, d1=20, center=false, $fn=160);
            }
            translate([30,0,-1]) cylinder(h=30, d=6.2, center=false, $fn=60);
        }
    }    

     // POE     // 15
    if(Part==15){
        translate([0,-8, 17.5]) rotate([180,0,0]) 
        POEboxTOP();
//        color([0,0,1]) rotate([180,0,90]) translate([4,1,-14.5]) import("poe-injector-05.stl", convexity = 5);
        POEbox();
    }

    // Stand        // 14
    if(Part==14 || Part<0){
        translate([0,-RODshift,-25]) Stand(20);
    }

    // rain cylinder    // 13
    if(Part==13){
        translate([0,0,256]) rotate([180,0,0]) RainCylinder();
    }else if(Part<0){
        RainCylinder();
    }

module RainCylinder(){
    difference(){
        hull(){
            translate([0,0-6, 256.6]) cylinder(h=1+5, d=100+2, center=false, $fn=200);
            translate([0,0,171.65]) cylinder(h=6, d=47+6, center=false, $fn=200);
            translate([40,-6, 171.65]) cylinder(h=6, d=20+6, center=false, $fn=200);
            translate([-40,-6, 171.65]) cylinder(h=6, d=20+6, center=false, $fn=200);
        }
        difference(){
            hull(){
                translate([0,0-6, 257.6]) cylinder(h=1+5, d=100, center=false, $fn=200);
                translate([0,0,177.5]) cylinder(h=8, d1=47-6, d2=41+7, center=false, $fn=200);
                translate([40,-6, 177.5]) cylinder(h=8, d=20-6, d2=14+7, center=false, $fn=200);
                translate([-40,-6, 177.5]) cylinder(h=8, d=20-6, d2=14+7, center=false, $fn=200);
            }
            CrosSupport();
        }
        rotate([180,0,0]) hull(){
                    translate([0,0,-177.6]) cylinder(h=6, d1=47.1-6, d2=47.1+6, center=false, $fn=100);
                    translate([40,6,-177.6]) cylinder(h=6, d1=20.1-6, d2=20.1+6, center=false, $fn=100);
                    translate([-40,6,-177.6]) cylinder(h=6, d1=20.1-6, d2=20.1+6, center=false, $fn=100);
        }
        translate([25,-20.5,178]) rotate([45,0,0]) M3(10);      //     rotate([180,0,0]) cylinder(h=LENGTH, d=3.2, $fn=30, center=false);
        translate([25,14.5,178]) rotate([-45,0,0]) M3(10);
        translate([-25,-20.5,178]) rotate([45,0,0]) M3(10);
        translate([-25,14.5,178]) rotate([-45,0,0]) M3(10);
    }
    // top for water
    difference(){
        hull(){
            translate([0,0-6, 256.6]) cylinder(h=1, d=100+2, center=false, $fn=200);
            translate([0,0-6, 221.1+1]) cylinder(h=1, d=5+4, center=false, $fn=200);
            translate([0,0-6, 221.1]) cylinder(h=1, d=4.5, center=false, $fn=200);
        }
        hull(){
            translate([0,0-6, 257.6]) cylinder(h=1, d=100, center=false, $fn=200);
            translate([0,0-6, 221]) cylinder(h=1, d=3, center=false, $fn=200);
        }
    }
}

module CrosSupport(){
        difference(){
            union(){
                translate([0,-6,177.6+40]) rotate([0,0,45]) cube([100,2,80], center=true);
                translate([0,-6,177.6+40]) rotate([0,0,-45]) cube([100,2,80], center=true);
            }
            hull(){
                translate([0,0-6, 256.7]) cylinder(h=1, d=100+2, center=false, $fn=200);
                translate([0,0-6, 221.1]) cylinder(h=1, d=5, center=false, $fn=200);
            }
            difference(){
                translate([0,-6,177.6+40]) cube([110,110,82], center=true);
                hull(){
                    translate([0,0-6, 256.6]) cylinder(h=1, d=100+2, center=false, $fn=200);
                    translate([0,0,171.65]) cylinder(h=6, d=47+6, center=false, $fn=200);
                    translate([40,-6, 171.65]) cylinder(h=6, d=20+6, center=false, $fn=200);
                    translate([-40,-6, 171.65]) cylinder(h=6, d=20+6, center=false, $fn=200);
                }
            }
            translate([0,-6,177+40]) cylinder(h=10, d1=36, d2=10, center=false, $fn=200);
            translate([0,-6,177]) cylinder(h=40.01, d1=50, d2=36, center=false, $fn=200);
        }
}

    // formatting bearing// 18
    if(Part==18){
        difference(){
            cylinder(h=10, d1=15.2, d2=14.8, center=false, $fn=300);
            translate([0,0,-1]) cylinder(h=12, d=3.8, center=false, $fn=30);
            translate([0,0,8]) cylinder(h=4, r=7/sin(60)/2, center=false, $fn=6);
        }
    }

    // water measure support    // 12
    if(Part==12){
        rotate([90,0,0]) supp4swing(3, 0, 23.5);
    }else if(Part<0){
        translate([0,-15.5,170.6+7]) supp4swing(3, 0, 23.5);     // hh, EXPAND, ZZ
    }

    // water measure    // 11
    if(Part==11){
        WaterMeasure(90, 15, 24-2, 14.5+2.5);
    }else if(Part<0){
        translate([0,13-19,201.15]) 
        rotate([0,25.75,0]) WaterMeasure(90, 15, 24-2, 14.5+2.5);
    }
    
    // rain base  // 10
    if(Part==10){
        rotate([0,180,0])
        RainBase();
    }else if(Part<0){
        translate([0,0,170.6])  RainBase();
    }
    
    // wind direction  // 8-9
    if(Part==9){
        rotate([0,180,0]) Out(40, 1.5, 2, 2);    // ZZ, MINUS, WALL, [2]-direction
    }else if(Part<0){
        translate([0,0,140])  Out(40, 1.5, 2, 2);    // ZZ, MINUS, WALL, [2]-direction
    }
    if(Part==8){
            rotate([180,0,90])  WindDir(40);
    }else if(Part<0){
        translate([0,3.0,137])  WindDir(40);
    }
    
    // top bearing in //7
    if(Part==7 || Part<0){
        translate([0,0,140])  In(40, 0.4, 1.6, BearingInTuning);    // ZZ, MINUS
    }
    // wind speed  //6
    if(Part==6){
        rotate([0,-90,0]) AnemoThird(45, 1.0);  // DIA, WALL
        translate([46,0,0]) rotate([0,-90,0]) AnemoThird(45, 1.0);  // DIA, WALL
        translate([23,68,0]) rotate([0,-90,180]) AnemoThird(45, 1.0);  // DIA, WALL
    }else if(Part<0){
        translate([0,0,100]) for(ROT = [0:120:360]){
            rotate([0,0,ROT]) 
            %AnemoThird(45, 1.0);  // DIA, WALL
        }
    }
    
    // bottom bearing out  //5
    if(Part==5){
        rotate([180,0,0]) Out(40, 1.5, 2, 1);    // ZZ, MINUS, WALL, [2]-direction
    }else if(Part<0){
        translate([0,0,100]) Out(40, 1.5, 2, 1);    // ZZ, MINUS, WALL, [2]-direction
    }
    
    // bottom bearing in  //4
    if(Part==4 || Part<0){
        translate([0,0,100]) In(40, 0.4, 0, BearingInTuning);    // ZZ, MINUS, PCBSHIFT, BT
    }

    // PCB base  //3
    if(Part==3 || Part<0){
        // TOP
        difference(){
            union(){
                translate([0,-RODshift,88.9]) rotate([90,0,90]) cylinder(h=15, d=6.2, $fn=4, center=true);
                translate([0,0,76]) scale([1,0.93,1]) cylinder(h=12.9, d1=28, d2=21, $fn=150);
                translate([0, 3+2, 75.5]) scale([1,0.5,1]) cylinder(h=2.5, d1=72, d2=68, $fn=100);
                translate([0, 3+2, 75.5-15.1]) scale([1,0.5,1]) cylinder(h=15.2, d=72, $fn=100);
            }
            translate([-8,2,0]) cube([16,3.5,90],center=false);
                translate([-8-3.2,2,74]) rotate([0,45,0]) cube([16,3.5,16],center=false);
            translate([-4,2,0]) cube([8.5, 7, 90],center=false);
           translate([0,-RODshift, -6]) cylinder(h=250, d=6.2, $fn=30, center=false);
            // ROOF
            hull(){
                translate([-50, 4+4, 75.4+0.1]) rotate([-135,0,0]) cube([100,34,34], center=false);
                translate([-50, 0, 75.4+0.1]) rotate([-135,0,0]) cube([100,34,34], center=false);
            }
//            translate([0, 4+4, 46]) cube([100,50,34], center=true);
        }
    }

//        // TOP - flat version
//        difference(){
//            union(){
//                translate([0,-RODshift,88.9]) rotate([90,0,90]) cylinder(h=15, d=6.2, $fn=4, center=true);
//                translate([0,0,76]) scale([1,0.93,1]) cylinder(h=12.9, d1=28, d2=21, $fn=150);
//                translate([0, 3+2, 75.5]) scale([1,0.5,1]) cylinder(h=2.5, d1=72, d2=68, $fn=100);
//            }
//            translate([-8,2,0]) cube([16,3.5,90],center=false);
//            translate([-4,2,0]) cube([8.5, 7, 90],center=false);
//           translate([0,-RODshift, -6]) cylinder(h=250, d=6.2, $fn=30, center=false);
//        // KEY
//        translate([0,-9.2,75.5]) rotate([90,0,90]) cylinder(h=15, d=6.0, $fn=4, center=true);
//                translate([0, 3+2, 75.3]) scale([1,0.5,1]) ring(70, 0.9, 0);
//        }

    // PCB base  //2
    if(Part==2 || Part<0){
//        EyeBase0();
        $fn=150;
        EyeBase1();
    }

    // PCB base  //1
    if(Part==1 || Part<0){
        difference(){
            translate([0, 3+2, -5]) scale([1,0.47,1]) cylinder(d=66+2-0.3, h=1.8, $fn=100, center=false);
        // ETH
        translate([-2.85, 10.3, 3])  cube([16+1,13.5+1,20],center=true);
        // ROD
        translate([0,-RODshift, 3]) cylinder(h=20, d=6.2, $fn=30, center=true);
        // external temp
        translate([15,5, 3]) cylinder(h=20, d=5.5, $fn=30, center=true);

        }
    }
}

module POEbox(){
    difference(){
        union(){
            hull(){
                translate([0, 0, -2]) cylinder(h=21.5, d=7, $fn=30, center=false);
                translate([37.5, 0, -2]) cylinder(h=21.5, d=7, $fn=30, center=false);
                translate([0, 35.5+0, -2]) cylinder(h=21.5, d=7, $fn=30, center=false);
                translate([37.5, 35.5+5, -2]) cylinder(h=21.5, d=7, $fn=30, center=false);
            }
            difference(){
                translate([39-12,-2,-2]) din();
                translate([30,17,8])  cube([14.5, 70, 25],center=true);
            }
        }
        hull(){
            translate([0, 0, 0]) cylinder(h=20, d=3, $fn=30, center=false);
            translate([37.5, 0, 0]) cylinder(h=20, d=3, $fn=30, center=false);
            translate([0, 35.5, 0]) cylinder(h=20, d=3, $fn=30, center=false);
            translate([37.5, 35.5+5, 0]) cylinder(h=20, d=3, $fn=30, center=false);
        }
        hull(){
            translate([32.5,35,20]) rotate([-90,0,0]) cylinder(d=10, h=10, $fn=60, center=false);
            translate([32.5,35,8]) rotate([-90,0,0]) cylinder(d=10, h=10, $fn=60, center=false);
        }
        translate([7.5,17,8])  cube([14.5, 50, 11.5],center=true);
        // SIGN
        translate([17.5, 41, 8]) {rotate([0,-90,-82]){
            linear_extrude(height = 2, center = false, convexity = 5, twist = 0, slices = 20, scale = 1.0) {
            text(text =  str("POE"), font = "Marsh:style=Stencil", halign="center", size=6);
        }}}
        // SIGN
        translate([17.5, -3, 8]) {rotate([180,90,90]){
            linear_extrude(height = 2, center = false, convexity = 5, twist = 0, slices = 20, scale = 1.0) {
            text(text =  str("ETH"), font = "Marsh:style=Stencil", halign="center", size=6);
        }}}
        }
        difference(){
            union(){
                hull(){
                    translate([20.5, 32.5, -1]) cylinder(h=13.9, d=8, $fn=30, center=false);
                    translate([20.5-4, 32.5+7, -1]) rotate([0,0,10]) cube([8,1,13.9]);
                }
                hull(){
                    translate([20.5, 3, -1]) cylinder(h=13.9, d=8, $fn=30, center=false);
                    translate([20.5-4, -3, -1]) cube([8,1,13.9]);
                }
            }
            translate([20.5, 32.5, 12.9-7]) cylinder(h=7.1, d1=2.6, d2=2.9, $fn=30, center=false);
            translate([20.5, 3, 12.9-7]) cylinder(h=7.1, d1=2.6, d2=2.9, $fn=30, center=false);
        }
}

module POEboxTOP(){
        difference(){
            union(){
                hull(){
                    translate([0, 0, 17.5]) cylinder(h=2, d=2.6, $fn=30, center=false);
                    translate([37.5, 0, 17.5]) cylinder(h=2, d=2.6, $fn=30, center=false);
                    translate([0, 35.5, 17.5]) cylinder(h=2, d=2.6, $fn=30, center=false);
                    translate([37.5, 35.5+5, 17.5]) cylinder(h=2, d=2.6, $fn=30, center=false);
                }
                    translate([20.5, 32.5, 14.5]) cylinder(h=5, d1=8, d2=12, $fn=30, center=false);
                    translate([20.5, 3, 14.5]) scale([1.5,1,1]) cylinder(h=5, d=8, $fn=30, center=false);
                    translate([33,39,14.5]) rotate([0,0,0]) cylinder(d=14+4, h=5, $fn=60, center=false);

            }
            translate([20.5, 32.5, 14]) cylinder(h=5, d=3.2, $fn=30, center=false);
                translate([20.5, 32.5, 16.5]) cylinder(h=5, d=6, $fn=30, center=false);
            translate([20.5, 3, 14]) cylinder(h=5, d=3.2, $fn=30, center=false);
                translate([20.5, 3, 16.5]) cylinder(h=5, d=6, $fn=30, center=false);
            translate([33,39,14]) rotate([0,0,0]) cylinder(d=14, h=10, $fn=60, center=false);
                translate([38.8, 30, -3]) cube([10,20,30]);
                translate([17.0, 39.08, -3]) rotate([0,0,7.55]) cube([30,10,30]);
            // Sign
            translate([35, 18, 19.2]) {rotate([0,0,90]){
                linear_extrude(height = 2, center = false, convexity = 5, twist = 0, slices = 20, scale = 1.0) {
                text(text =  str("9-24V"), font = "Marsh:style=Stencil", halign="center", size=6);
            }}}
            translate([8.5, 29, 19.2]) {rotate([0,0,-0]){
                linear_extrude(height = 2, center = false, convexity = 5, twist = 0, slices = 20, scale = 1.0) {
                text(text =  str("POE"), font = "Marsh:style=Stencil", halign="center", size=6);
            }}}
            translate([8.5, 1, 19.2]) {rotate([0,0,-0]){
                linear_extrude(height = 2, center = false, convexity = 5, twist = 0, slices = 20, scale = 1.0) {
                text(text =  str("ETH"), font = "Marsh:style=Stencil", halign="center", size=6);
            }}}

        }
}

module din(){
    MountpointDistance =0;
    Identation = 0;
    minwidth = 21.5;    // width without Z axis shift mount hole 2 (top)
    screwSmall = 2.7;
    sign = "";
    fontsize=5;
    fanrot=0;       //do not change
    // only positive values !
    z2shift = 0;    // Z axis shift mount hole 2 (top)  move R -->
    z1shift = 0;    // Z axis shift mount hole 1 (bottom)  move R -->
    y1shift = 0;          // Y axis shift mount hole 1 (bottom) move UP ^

    //----------------------------------- 
    $fn=50;
    long = 45;
    expand = 1;      // 0-x [default 1]
    width = minwidth+z2shift+z1shift;
	difference() {
		union() {
			hull() {
				translate([15+expand,long-2,0]) {cylinder(h=width, r=2, center=false);}
                translate([12+expand,37.5,0]) {cube([5,1,width], center=false);}
                translate([12+expand,long-1,0]) {cube([1,1,width], center=false);}
            }
			hull() {
				translate([16+expand,35.7,0]) {cylinder(h=width, r=1, center=false);}
                translate([14.2+expand,37.5,0]) {cube([2.8,1,width], center=false);}
            }
			hull() {
				translate([18.5+expand,0.5-1,0]) {cylinder(h=width, r=0.5, center=false);}
				translate([14.5+expand,3.8,0]) {cylinder(h=width, r=0.3, center=false);}
				translate([14.2+expand,0-1,0]) {cube([1,1,width], center=false);}
			}
			translate([0,-1,0]) cube([3.5, 4+1, width], center=false);
			translate([0,-1,0]) cube([15+expand, 2+1, width], center=false);
			hull() {
				translate([12+expand,4,0]) {cylinder(h=width, r=1, center=false);}
                translate([0,3,0]) {cube([1,1,width], center=false);}
                translate([0,long-1,0]) {cube([13+expand,1,width], center=false);}
            }
			hull() {
            translate([0,40,0]) {cube([14,1,width], center=false);}
            translate([2,MountpointDistance+14+y1shift,0]) {cylinder(h=width, r=2, center=false);}
            translate([6,MountpointDistance+14+y1shift,0]) {cylinder(h=width, r=2, center=false);}
            }
            hull(){
                translate([-Identation,10.5-3.5+y1shift,width-7-z2shift]) {cube([Identation+1,7,7], center=false);}
                translate([0,10.5-3.5+y1shift,width-7-z1shift]) {cube([1,7,7], center=false);}
            }
            hull(){
                translate([-Identation,10.5-3.5+MountpointDistance+y1shift,width-7-z1shift]) {cube([Identation+1,7,7], center=false);}
                translate([0,10.5-3.5+MountpointDistance+y1shift,width-7-z2shift]) {cube([1,7,7], center=false);}
            }
        }
        //-
        translate([3.5,2.5,-1]) {cylinder(h=width+2, r=0.5, center=false);}
        // - Aretation M3
        translate([9+expand,-1-1,width/2+z2shift/2]) rotate([-90,0,0]){cylinder(h=16, d=screwSmall, center=false);}
        translate([9+expand,2.9,width/2+z2shift/2]) rotate([-90,0,0]){cylinder(h=3, d1=screwSmall, d1=3.2, center=false);}
        translate([9+expand,-1-1,width/2+z2shift/2]) rotate([-90,0,0]){cylinder(h=4.5, d=3.4, center=false);}
        // - MountPoint1
        translate([-1-Identation,10.5+y1shift,width/2-z2shift/2+z1shift/2]) rotate([0,90,0]){cylinder(h=9+expand+Identation+6, d=screwSmall, center=false);}
        translate([-0.1-Identation,10.5+y1shift,width/2-z2shift/2+z1shift/2]) rotate([0,90,0]){cylinder(h=4, d1=3.2, d2=screwSmall, center=false);}
        // - MountPoint2
       translate([-1-Identation,10.5+MountpointDistance+y1shift,width/2+z2shift/2-z1shift/2]) rotate([0,90,0]){cylinder(h=11+expand+Identation, d=screwSmall, center=false);}
       translate([-0.1-Identation,10.5+MountpointDistance+y1shift,width/2+z2shift/2-z1shift/2]) rotate([0,90,0]){cylinder(h=4, d1=3.2, d2=screwSmall, center=false);}
         // - SIGN
        translate([7+expand/2,16+y1shift,width-1]){
            linear_extrude(height = 3, center = false, convexity = 5, twist = -fanrot, slices = 20, scale = 1.0) {
               text(str(sign), font = "Sans Uralic:style=Bold", halign="center", size=fontsize);
            }
        }
    }
}

    // PCB and bearing  // -1
    if(Part<0){
        // Bearings
        color([0,1,0])  translate([0, 0, 30+40+40+30]) rotate([0,90,0]) import("61802.stl", convexity = 5);
        color([0,1,0])  translate([0, 0, 100]) rotate([0,90,0]) import("61802.stl", convexity = 5);
        color(c=[1,1,1,1]) translate([0,-RODshift, -4]) rotate([0,0,0]) cylinder(h=190, d=6, $fn=30, center=false);
        color(c=[1,1,1,1]) translate([0,-RODshift,180]) cylinder(h=3.9, r=5/cos(30), $fn=6, center=false);
        color(c=[1,1,1,1]) translate([0,-RODshift,-3]) cylinder(h=3.9, r=5/cos(30), $fn=6, center=false);
        // PCB
        color([0,0,1]) translate([0,1.9,0]) rotate([90,0,180]) translate([123-33, -170, 0]) import("pcb-wx-07.stl", convexity = 5);
        color([0,0,1]) translate([170.3, -90.2, 170.6]) rotate([180,0,-90]) import("pcb-wx-06-COIN.stl", convexity = 5);
    }

module EyeBase1(){
        // inside eye
        difference(){
            union(){
                translate([0, 3+2, -5]) scale([1,0.5,1]) cylinder(d=66+6, h=80.4, $fn=50);
//                translate([0, 3+2, 75.3]) scale([1,0.5,1]) ring(70-0.4, 0.7, 0);
            }
            // ROOF
            translate([-50, 4+4, 75.4]) rotate([-45,0,0]) cube([100,34,34], center=false);
            translate([-50, 0, 75.4]) rotate([135,0,0]) cube([100,34,34], center=false);

            // in
            difference(){
                translate([0, 3+2, -6]) scale([1,0.47,1]) cylinder(d=66+2, h=83, $fn=50);
                // top cut
                difference(){
                    translate([0,28,75.5]) rotate([45,0,0]) cube([100,34,34],center=true);
                    translate([-8,2,0]) cube([16,3.5,90],center=false);
                    translate([-4,2,0]) cube([8.5, 7, 90],center=false);
                    // ETH
                    translate([-2.85, 10.3, 8])  cube([16+4,13.5+3,200],center=true);
                    // PCB
                    translate([-1, 5, 8])  cube([64,5,200],center=true);
                    translate([-25.5, 7.5, 8])  cube([13.5,5,200],center=true);
                    translate([21.3, 8-1.5, 8])  cube([7,13,200],center=true);
                }
                //-half
                difference(){
                    translate([-50,-30+6,9]) cube([100,30,90],center=false);
                    translate([-31-1.5,1.5,0]) cube([62+1.5,10,90],center=false);
                        translate([-33.5,1.5,0]) cube([67,10,40],center=false);
                    translate([-33.5,2,0]) cube([67,2,90],center=false);
                        translate([18,0,0]) cube([13,4,90],center=false);
                    translate([31,3, -6]) cylinder(h=40, d=10, $fn=30, center=false);
                        translate([31,3, 34]) sphere(d=10, $fn=30);
                    translate([28,1.5,31.3]) rotate([0,-45,0]) cube([10,5,10],center=false);
                    translate([-28,1.5,34]) rotate([0,-45,0]) cube([10,5,10],center=false);
                }
            }
            translate([0,-RODshift, -6]) cylinder(h=250, d=6.2, $fn=30, center=false);
        }
        // KEY
//        translate([0,-9.2,75.4]) rotate([90,0,90]) cylinder(h=14, d=6.0, $fn=4, center=true);

        // outside eye
        difference(){
            hull(){
                translate([0, 3+2, 70+5]) scale([1,0.6,1]) ring(87, 10, 0,100);
                translate([0, 3+2, -5]) scale([1, 0.6, 1]) cylinder(d=66+6+15, h=80.4-10.4, $fn=200);
                translate([0, -21, -5]) scale([1.5,1,1]) cylinder(d=MountDia, h=65, $fn=100);
            }
            // space 4 windspeed
            translate([0,0,76]) scale([1,0.93,1]) cylinder(h=15, d1=45, d2=55, $fn=50);

            // mounting vertical/horizontal
            translate([0,-21-MountDia/2,-6]) rotate([0,0,0]) cylinder(d=MountDia, h=70+50, $fn=MountDiaFn, center=false);  
            translate([0,-21-MountDia/2,30]) rotate([0,90,0]) cylinder(d=MountDia, h=MountDia*3, $fn=MountDiaFn, center=true);  
            // binder
            translate([0,-21-MountDia/2,60]) Binder(MountDia);
            translate([0,-21-MountDia/2,0]) Binder(MountDia);
            translate([-MountDia/2-4,-21-MountDia/2,30]) rotate([0,90,0]) Binder(MountDia);
            translate([MountDia/2+4,-21-MountDia/2,30]) rotate([0,90,0]) Binder(MountDia);
            // ROD
           translate([0,-RODshift, -6]) cylinder(h=250, d=6.2, $fn=30, center=false);
            translate([-8,2,0]) cube([16,3.5,90],center=false);
            translate([-4,2,0]) cube([8.5, 7, 90],center=false);
            // in
            translate([0, 3+2, 70+5-1]) scale([1,0.58,1]) ring(87-4, 10, 0,30);
            translate([0, 3+2, -6]) scale([1, 0.58, 1]) cylinder(d=66+2+15, h=70+6+5, $fn=50);
            translate([0, 3+2, -5]) scale([1,0.5,1]) cylinder(d=72+4, h=100, $fn=50);

            // N
            translate([43.0,1.5,20]) {rotate([0,90,0]){
                linear_extrude(height = 2, center = false, convexity = 5, twist = 0, slices = 20, scale = 1.0) {
                text(text =  "NORTH", font = "Marsh:style=Stencil", halign="center", size=7);
            }}}
            // S
            translate([-43.0,1.5,20]) {rotate([0,-90,0]){
                linear_extrude(height = 2, center = false, convexity = 5, twist = 0, slices = 20, scale = 1.0) {
                text(text =  "SOUTH", font = "Marsh:style=Stencil", halign="center", size=7);
            }}}
        }
        // vyztuhy
        difference(){
            union(){
                translate([0,5,40-5-6.25])  rotate([0,0,35]) cube([68,1,80-12.5],center=true);
                translate([0,5,40-5-6.25])  rotate([0,0,-35]) cube([68,1,80-12.5],center=true);
                translate([0,-16,40-5-6.5])  cube([2,8,80-13],center=true);
            }
          translate([0, 3+2, -6]) scale([1,0.5,1]) cylinder(d=66+6, h=100, $fn=50);  
            translate([0,-21-MountDia/2,60]) Binder(MountDia);
            translate([0,-21-MountDia/2,0]) Binder(MountDia);
        }
        BottomEye();
    }

module EyeBase0(){
        // inside eye
        difference(){
            union(){
                translate([0, 3+2, -5]) scale([1,0.5,1]) cylinder(d=66+6, h=80.4, $fn=100);
                translate([0, 3+2, 75.3]) scale([1,0.5,1]) ring(70-0.4, 0.7, 0);
            }
            // screwdriver
//            translate([9.2,5,14]) rotate([-90,0,0]) cylinder(h=30, d=3.5, $fn=30, center=false);
//            translate([-14.8,5,14]) rotate([-90,0,0]) cylinder(h=30, d=3.5, $fn=30, center=false);
            // in
            difference(){
                translate([0, 3+2, -6]) scale([1,0.47,1]) cylinder(d=66+2, h=83, $fn=100);
                // top cut
                difference(){
                    translate([0,28,75.5]) rotate([45,0,0]) cube([100,34,34],center=true);
                    translate([-8,2,0]) cube([16,3.5,90],center=false);
                    translate([-4,2,0]) cube([8.5, 7, 90],center=false);
                    // ETH
                    translate([-2.85, 10.3, 8])  cube([16+4,13.5+3,200],center=true);
                    // PCB
                    translate([-1, 5, 8])  cube([64,5,200],center=true);
                    translate([-25.5, 7.5, 8])  cube([13.5,5,200],center=true);
                    translate([21.3, 8-1.5, 8])  cube([7,13,200],center=true);

                }
                //-half
                difference(){
                    translate([-50,-30+6,9]) cube([100,30,90],center=false);
                    // M3
//                    translate([9.2,2,14]) rotate([90,0,0]) cylinder(h=3.1, d1=3.2, d2=2.7, $fn=30, center=false);
//                        translate([9.2,-1,14]) rotate([90,0,0]) cylinder(h=12, d=2.7, $fn=30, center=false);
//                    translate([-14.8,2,14]) rotate([90,0,0]) cylinder(h=3.1, d1=3.2, d2=2.7, $fn=30, center=false);
//                        translate([-14.8,-1,14]) rotate([90,0,0]) cylinder(h=12, d=2.7, $fn=30, center=false);

                    translate([-31-1.5,1.5,0]) cube([62+1.5,10,90],center=false);
                        translate([-33.5,1.5,0]) cube([67,10,40],center=false);
                    translate([-33.5,2,0]) cube([67,2,90],center=false);
//                        translate([-12,0,0]) cube([13,4,90],center=false);
                        translate([18,0,0]) cube([13,4,90],center=false);
                    translate([31,3, -6]) cylinder(h=40, d=10, $fn=30, center=false);
                        translate([31,3, 34]) sphere(d=10, $fn=30, center=false);
                    translate([28,1.5,31.3]) rotate([0,-45,0]) cube([10,5,10],center=false);
                    translate([-28,1.5,34]) rotate([0,-45,0]) cube([10,5,10],center=false);
                }
            }
            translate([0,-RODshift, -6]) cylinder(h=250, d=6.2, $fn=30, center=false);
        }
        // KEY
        translate([0,-9.2,75.4]) rotate([90,0,90]) cylinder(h=14, d=6.0, $fn=4, center=true);

        // outside eye
        difference(){
            union(){
//                translate([0, 3+2, 75]) scale([1,0.6, 0.1]) sphere(d=66+6+15, $fn=100);
                hull(){
                    translate([0, 3+2, -5]) scale([1, 0.6, 1]) cylinder(d=66+6+15, h=80.4+15, $fn=200);
                    translate([0, -21, -5]) scale([1.5,1,1]) cylinder(d=MountDia, h=65, $fn=100);
                }
            }
            // space 4 windspeed
            translate([0,0,76]) scale([1,0.93,1]) cylinder(h=15, d1=45, d2=55, $fn=150);

            // screwdriver
//            translate([9.2,5,14]) rotate([-90,0,0]) cylinder(h=30, d=3.5, $fn=30, center=false);
//            translate([-14.8,5,14]) rotate([-90,0,0]) cylinder(h=30, d=3.5, $fn=30, center=false);

            // mounting vertical/horizontal
            translate([0,-21-MountDia/2,-6]) rotate([0,0,0]) cylinder(d=MountDia, h=70+50, $fn=MountDiaFn, center=false);  
            translate([0,-21-MountDia/2,30]) rotate([0,90,0]) cylinder(d=MountDia, h=MountDia*3, $fn=MountDiaFn, center=true);  
            // binder
            translate([0,-21-MountDia/2,60]) Binder(MountDia);
            translate([0,-21-MountDia/2,0]) Binder(MountDia);
            translate([-MountDia/2-4,-21-MountDia/2,30]) rotate([0,90,0]) Binder(MountDia);
            translate([MountDia/2+4,-21-MountDia/2,30]) rotate([0,90,0]) Binder(MountDia);
            // ROD
           translate([0,-RODshift, -6]) cylinder(h=250, d=6.2, $fn=30, center=false);
            translate([-8,2,0]) cube([16,3.5,90],center=false);
            translate([-4,2,0]) cube([8.5, 7, 90],center=false);
            // in
            translate([0, 3+2, -6]) scale([1, 0.58, 1]) cylinder(d=66+2+15, h=100, $fn=100);
            // vetrani
//            for(j = [0 : 45/4 : 180]){
//                 translate([0,5,70]) rotate([90,0,j]) scale([0.4,1,1]) cylinder(d=10, h=100, $fn=30, center=true);
//            }
            // N
            translate([43.0,1.5,20]) {rotate([0,90,0]){
                linear_extrude(height = 2, center = false, convexity = 5, twist = 0, slices = 20, scale = 1.0) {
                text(text =  "NORTH", font = "Marsh:style=Stencil", halign="center", size=7);
            }}}
            // S
            translate([-43.0,1.5,20]) {rotate([0,-90,0]){
                linear_extrude(height = 2, center = false, convexity = 5, twist = 0, slices = 20, scale = 1.0) {
                text(text =  "SOUTH", font = "Marsh:style=Stencil", halign="center", size=7);
            }}}

        }
        // vyztuhy
        difference(){
            union(){
                translate([0,5,40-5])  rotate([0,0,35]) cube([68,1,80],center=true);
                translate([0,5,40-5])  rotate([0,0,-35]) cube([68,1,80],center=true);
                translate([0,-16,40-5])  cube([2,8,80],center=true);
            }
          translate([0, 3+2, -6]) scale([1,0.5,1]) cylinder(d=66+6, h=100, $fn=100);  
            translate([0,-21-MountDia/2,60]) Binder(MountDia);
            translate([0,-21-MountDia/2,0]) Binder(MountDia);
        }
        BottomEye();
    }

module BottomEye(){
    difference(){
        translate([0, 3+2, -2]) scale([1,0.47,1]) cylinder(d=66+2-0.3+0.5, h=11.8+0.21-1, $fn=100, center=false);
        // half
        translate([-50,1.5,-4]) cube([100,14,12.8-2],center=false);
            translate([8,-15,-4]) cube([22,30,12.8-2],center=false);
        // ETH
        translate([-2.85, 10.3, 3])  cube([16+1,13.5+1,20],center=true);
            translate([-11.5,13,-4]) cube([17.3,10,12.8-2],center=false);
        // USB-C
        translate([-17.85-3.2, 3.45, 3])  cube([12+7.0, 4,40],center=true);
            translate([-17.85-3.2, 4.5, 3])  cube([10, 5,40],center=true);
        // ROD
        translate([0,-RODshift, 3]) cylinder(h=20, d=6.2, $fn=30, center=true);
//        translate([0,-RODshift, -3]) cylinder(h=6, r=10/sin(60)/2, center=false, $fn=6);
        translate([0,-RODshift, -3]) cylinder(h=6, d=25, center=false, $fn=60);
        // TEMP
        hull(){
            translate([26, 3.3-1, 3])  cube([16, 3.5, 20],center=true);
            translate([26, 3.3+1, 3])  cube([16-2, 3.5, 20],center=true);
        }
            translate([15,5, 3]) cylinder(h=20, d=5.5, $fn=30, center=true);
    }
}


module MountTerminal(DD){
    difference(){
        translate([(DD+4)/2,-19,-5]) rotate([0,0,180]) cube([DD+4,DD/2,DD+4], center=false);
        translate([0,-21-DD/2,(DD+4)/2-5]) rotate([0,0,0]) cylinder(d=DD, h=DD+10, $fn=100, center=true);  
        translate([0,-21-DD/2,(DD+4)/2-5]) rotate([0,90,0]) cylinder(d=DD, h=DD+10, $fn=100, center=true);  
    }
}

module ring(prumer, zaobleni, Xplus, FN){
  rotate_extrude(convexity = 1) {
        translate([prumer/2-zaobleni+Xplus, 0, 0])
        hull(){
            circle(r = zaobleni, $fn=FN);
            translate([-Xplus,0,0]) circle(r = zaobleni, $fn=FN);
        }
    }
}

module WaterMeasure(XX,YY,ZZ,MagnetZZ){
    REDUCE=0.5;
    translate([0,0,-3.7])  difference(){
        union(){
            difference(){
                hull(){
                    translate([0,0,ZZ/2]) cube([1,YY,ZZ], center=true);
//                    cube([XX,YY,1], center=true);
                    cube([1,YY,1], center=true);
                    translate([-XX/2,0,0]) cube([0.1,YY*REDUCE,1], center=true);
                        translate([-XX/4,0,ZZ/4-0.5]) cube([0.1,YY,ZZ/2], center=true);
                    translate([XX/2,0,0]) cube([0.1,YY*REDUCE,1], center=true);
                        translate([XX/4,0,ZZ/4-0.5]) cube([0.1,YY,ZZ/2], center=true);
                }
//                translate([0,0,ZZ/2+0.5]) cube([XX+2,YY-2,ZZ], center=true);
                hull(){
                    translate([-XX/2,0,ZZ/2+0.5]) cube([1,YY*REDUCE-2,ZZ], center=true);
                        translate([-XX/4,0,ZZ/2+0.5]) cube([1,YY-2,ZZ], center=true);
                    translate([0,0,ZZ/2+0.5]) cube([1,YY-2,ZZ], center=true);
                    translate([XX/2,0,ZZ/2+0.5]) cube([1,YY*REDUCE-2,ZZ], center=true);
                        translate([XX/4,0,ZZ/2+0.5]) cube([1,YY-2,ZZ], center=true);
                }
            }
            hull(){
                translate([0,0,ZZ-0.5]) cube([1,YY,1], center=true);
                translate([0,0,0]) cube([XX/2.3,YY,1], center=true);
            }
            // magnet base
            translate([0,5,MagnetZZ]) rotate([90,0,0]) cylinder(h=5, d=5, $fn=30, center=true);
        }
    // M2
    translate([0,0,3.7]) rotate([90,0,0]) cylinder(h=17, d=1.8, $fn=30, center=true);
    // magnet hole
    color(c=[1,0,0,1]) translate([0,6,MagnetZZ]) rotate([90,0,0]) cylinder(h=3.2, d=5, $fn=30, center=true);
    }
}

module M3(LENGTH){
    translate([0,0,-0.01]) cylinder(h=30, d=6.2, $fn=30, center=false);
    rotate([180,0,0]) cylinder(h=LENGTH, d=3.2, $fn=30, center=false);
    
}
//-----------------------------------------------------------------------------------------------





module Build(NR){
    if(NR==16){
        for(i= [-30 : 10 : 30]){
            translate([i*2,25,0]) In(40, 0.4, 1.6, i);    // ZZ, MINUS    // 10
            translate([i*2,0,0]) In(40, 0.4, 0, i);    // ZZ, MINUS    // 07
        }
    }    
//    if(NR<0){
         // ROD
        color([1,1,1]){
//            translate([0,-RODshift, 150+5]) rotate([180,0,0]) cylinder(h=250, d=6, $fn=30, center=false);
//            translate([0,-RODshift,149.5]) cylinder(h=5, r=5/cos(30), $fn=6, center=false);
        }
        // Bearings
//        color([0,1,0])  translate([0, 0, 30+40+40+30]) rotate([0,90,0]) import("61802.stl", convexity = 5);
//        color([0,1,0])  translate([0, 0, 30+40+30]) rotate([0,90,0]) import("61802.stl", convexity = 5);
        // PCB
//        color([0,0,1]) translate([0,1.9,0]) rotate([90,0,180]) translate([123-33, -170, 0]) import("pcb-wx-06.stl", convexity = 5);
//        color([0,0,1])  translate([90.15, 147.5+PCBshift, 140.6]) rotate([180,0,0]) import("pcb-wx-06-COIN.stl", convexity = 5);
//    }


//#cube([10,20,87]);
    // Rain
    translate([0, 0, 30+40+30+40+3+1.6+5]) {
        // 1mm rain = 78ml
        if(NR==888){
            %rotate([180,0,-38.5]) RainCover(104,32+5);    // 14
        }else if(NR==14){
            rotate([180,0,-8.5-30]) RainCover(104,32+5);    // 14
        }

        if(NR==88){
//            translate([10,0,15]) swing2(45, 20, 20);    // 13
            translate([0,0,20-3]) swing3(45, 20, 20);    // 13
        }else if(NR==13){
            translate([0,0,3+7+5]) rotate([-90,0,0]) swing(45, 20, 15+10);    // 13
        }

//        if(NR<0){
       if(NR==88){
            %translate([0,12+6,0]) supp4swing(3, 0, 20-3);      // 12b
            %translate([0,-12-6,0]) supp4swing(3, 0, 20-3);      // 12b
            %RainBase();    // 12
        }else if(NR==12){
            rotate([90,0,0]) translate([45,1.5,0]) supp4swing(3, 0);      // 12b
            rotate([90,0,0]) translate([-45,1.5,0]) supp4swing(3, 0);      // 12b
            rotate([180,0,0]) RainBase();    // 12
        }
    }

    // Wind Direction
    translate([0,0,30+40+40]){
//       if(NR<0){
       if(NR<0){
             %Out(40, 1.5, 2, 2);    // ZZ, MINUS, WALL, [2]-direction    // 11
            % translate([0,3,0]) WindDir(40);
            // Magnet
            color([1,0,0]) translate([0,-13-2,26-2]) cube([12.2, 6.2, 2.2], center=true);

//           translate([0, -13.5, 26-10]) cylinder(h=10, d=6, $fn=30, center=false); 
        }else if(NR==11){
            rotate([0,180,0]) Out(40, 1.5, 2, 2);    // ZZ, MINUS, WALL, [2]-direction    // 11
            translate([20,-80,-4]) rotate([0,180,0]) WindDir(40);
        }
        
        if(NR<0 || NR==10){
           translate([0,0,30])  In(40, 0.4, 1.6, BearingInTuning);    // ZZ, MINUS    // 10**
        }
    }

    // Anemometer
    translate([0,0,30+40]){
        if(NR<0){
            for(ROT = [0:120:360]){
                rotate([0,0,ROT]) 
                %AnemoThird(45, 1.0);  // DIA, WALL    // 09
            }
        }else if(NR==9){
            rotate([0,-90,0]) AnemoThird(45, 1.0);  // DIA, WALL    // 09
        }
        
        if(NR<0 || NR==7){
//            %Out(40, 1.5, 2, 1);    // ZZ, MINUS, WALL, [2]-direction    // 08
        }else if(NR==8){
//            rotate([0,180,0]) Out(40, 1.5, 2, 1);    // ZZ, MINUS, WALL, [2]-direction    // 08
        }
        
        if(NR<0 || NR==7){
//            translate([0,0,30]) In(40, 0.4, 0, BearingInTuning);    // ZZ, MINUS, PCBSHIFT, BT    // 07**
        }        
    }

    // sun guard
    translate([0,0,0]){
        if(NR<0){
            %translate([0,0,32]) SunGuard(80, 4, 0);    // 06
        }else if(NR==6){
            translate([0,0,32+30]) SunGuard(80, 4, 0);    // 06 **
        }

        if(NR<0){
            %translate([0,0,24]) SunGuard(80, 3, 0);    // 05
        }else if(NR==5){
            rotate([180,0,0]) translate([0,0,24]) SunGuard(80, 3, 0);    // 05
        }

        if(NR<0){
            %translate([0,0,16]) SunGuard(80, 2, 0);    // 04
        }else if(NR==4){
            rotate([180,0,0]) translate([0,0,16]) SunGuard(80, 2, 0);    // 04
        }

        if(NR<0){
            %translate([0,0,8]) SunGuard(80, 2, 0);    // 03
        }else if(NR==3){
            rotate([180,0,0]) 
            translate([0,0,8]) SunGuard(80, 2, 0);    // 03
        }

        if(NR<0){
            %translate([0,0,0]) SunGuard(80, 1, 80);    // 02
        }else if(NR==2){
            // TOP
            difference(){
                rotate([180,0,0]) translate([0,0,0]) SunGuard(80, 1, 80);    // 02
                translate([0,0,0.01]) rotate([180,0,0]) cylinder(h=80, d=90, center=false, $fn=30);
                translate([-22,3,0]) rotate([45,0,0]) cube([17,5,5], center=true);
                translate([22,3,0]) rotate([45,0,0]) cube([17,5,5], center=true);
            }
        }
            
        if(NR==1){
            // BOT
            difference(){
                rotate([180,0,0]) translate([0,0,0]) SunGuard(80, 1, 80);    // 01
                translate([0,0,0]) cylinder(h=80, d=90, center=false, $fn=30);
            }
            translate([-22,3,-0.2]) rotate([45,0,0]) cube([15,5,5], center=true);
            translate([22,3,-0.2]) rotate([45,0,0]) cube([15,5,5], center=true);
        }

        if(NR<0){
            %translate([0,0,0]) Bottom(75, 80);    // 00
        }else if(NR==0){
            translate([0,0,0]) Bottom(75, 80);    // 00
        }
    }

    if(NR<0){
        %translate([0,0,-100]) Stand(20);
    }else if(NR==15){
        translate([0,0,-100]) Stand(20);
    }
}

module Stand(ZZ){
    difference(){
        union(){
            for (x = [0:120:360]){
                hull(){
                    translate([0,0,0]) cylinder(h=ZZ, d=15, center=false, $fn=30);
                    rotate([0,0,x]) translate([100,0,0]) cylinder(h=2, d=15, center=false, $fn=30);
                }
            }
        }
        // -ROD
        translate([0,0,-0.1]) cylinder(h=ZZ+0.2, d1=5.5, d2=6.2, center=false, $fn=30);
    }
}

module M(MM, LONG){
    rotate([90,0,0]) cylinder(h=LONG, d1=MM+0.2, d2=MM*0.85, center=false, $fn=30);
    rotate([-90,0,0]) cylinder(h=LONG*2, d=MM*2, center=false, $fn=30);
}

module RainCover(DIA, DEPTH){
    difference(){
        union(){
            // top ring
            difference(){
                translate([0,0,-2-60]) rotate([180,0,0]) cylinder(h=10, r=DIA/2, center=false, $fn=200);
                translate([0,0,-2-60+0.1]) rotate([180,0,0]) cylinder(h=10.2, r1=DIA/2-2, r2=DIA/2-0.5, center=false, $fn=200);
            }
            // OUT
            difference(){
                translate([0,0,-2]) rotate([180,0,0]) cylinder(h=60, r1=diameter/2+tloustka+space-1.75, r2=DIA/2, center=false, $fn=200);
                translate([0,0,-1.9]) rotate([180,0,0]) cylinder(h=60.2, r1=diameter/2+tloustka+space-1-2.75, r2=DIA/2-2, center=false, $fn=200);
            // -RQ
//            translate([25, 32.5, -20]) rotate([128,65,0]) rotate([0,0,-90]) scale([1.8, 1.8, 1]) rq(2);
            }

            // IN
            difference(){
                translate([0,0,-2-DEPTH]) rotate([180,0,0]) cylinder(h=60-DEPTH, r1=6, r2=DIA/2, center=false, $fn=200);
                translate([0,0,-1.9-DEPTH]) rotate([180,0,0]) cylinder(h=60.2-DEPTH, r1=2, r2=DIA/2-2, center=false, $fn=200);
            }
            // H2O out
            difference(){
                translate([0,0,-2-DEPTH+10]) rotate([180,0,0]) cylinder(h=10, r1=0.9, r2=6, center=false, $fn=200);
                translate([0,0,-1.9-DEPTH+10]) rotate([180,0,0]) cylinder(h=10.2, r1=0.5, r2=2, center=false, $fn=200);
            }
            // fixate
            difference(){
                 for (x = [0:90:360]){
                    hull(){
                        translate([0,0,-2]) rotate([180,0,0]) cylinder(h=60, d=1, center=false, $fn=20);
                        rotate([0,0,x]) translate([DIA/2,0,-2]) rotate([180,0,0]) cylinder(h=60, d=1, center=false, $fn=20);
                     }
                }
                translate([0,0,-1.9]) rotate([180,0,0]) cylinder(h=60.2-DEPTH/2, r1=diameter/2+tloustka+space-1-2.75, r2=5, center=false, $fn=20);
                translate([0,0,-1.9-DEPTH+10]) rotate([180,0,0]) cylinder(h=10.2, r=3, center=false, $fn=20);
                translate([0,0,-1.9-DEPTH]) rotate([180,0,0]) cylinder(h=60.2-DEPTH, r1=3, r2=DIA/2-2, center=false, $fn=200);
                difference(){
                    translate([0,0,-2]) rotate([180,0,0]) cylinder(h=60, r=DIA/2+5, center=false, $fn=20);
                    translate([0,0,-1.9]) rotate([180,0,0]) cylinder(h=60.2, r1=diameter/2+tloustka+space-1.75, r2=DIA/2, center=false, $fn=20);
                }
            }

            // KEY lock
            difference() {
                union(){
                    hull(){
                        cylinder(h=4*tloustka, r=diameter/2+tloustka+space, center=false, $fn=200);
                        translate([0,0,-20]) sphere(d = 26+2*tloustka);
                    }
                    // expand for M3
                    translate([0,0,-2]) rotate([0,0,-38]) scale([1.2, 0.3, 1]) cylinder(h=4*tloustka+2, r=diameter/2+tloustka+space, center=false, $fn=50);
                }
                    translate([0,0,-2]) rotate([180,0,0]) cylinder(h=40, r=diameter/2+tloustka+space, center=false, $fn=200);
                hull(){
                    cylinder(h=5*tloustka, r=diameter/2+space, center=false, $fn=200);
                    translate([0,0,-28]) cylinder(h=0.1, r=13-tloustka, center=false);
                }
                // -M3
                translate([0,0,1.2]) rotate([0,90,-38]) cylinder(h=DIA, d=2.7, center=true, $fn=30);
                // -sign
                translate([0,0,8]) rotate([0,90,-38]) cylinder(h=DIA, d=5, center=true, $fn=3);
            }
             for (x = [0:30:360]){
             rotate([0,0,x]) translate([0,-diameter/2-0.7+space,3*tloustka-1.2]) rotate([15,0,0]) cube([5,tloustka,2*tloustka], center=true);
             }
         }
        // -cut
        if (CUT>0) {
            translate([-70,0,0]) cube([140, 10*100, 200], center=true);
        }
     }
}

module supp4swing(hh, EXPAND, ZZ){
    difference(){
        union(){
            hull(){
                translate([0,0,ZZ]) rotate([0,90, 90]) cylinder(h=hh+EXPAND, d=3+6, $fn=50, center=true);
                translate([0,0,0]) rotate([0,0, 0]) cube([14,hh+EXPAND,0.1], center=true);
            }
            translate([0,0,-2]) rotate([0,0, 0]) cube([10,hh+EXPAND,4], center=true);
            translate([0,0,-4.5]) rotate([0,0, 0]) cube([12+2*EXPAND,hh+EXPAND,1+2*EXPAND], center=true);
        }
        // -supp4swing
        //            translate([0,0,-3]) rotate([0,90, 90]) cylinder(h=hh+1, d=3+6+15, $fn=50, center=true);
        translate([0,0,ZZ]) rotate([0,90, 90]) cylinder(h=hh+EXPAND+1, d=2.5, $fn=50, center=true);
        if(EXPAND==0){
            translate([0,0,6]) rotate([0,90, 90]) cylinder(h=hh+EXPAND+1, d=3, $fn=50, center=true);
            translate([0,0,0]) rotate([0,0, 0]) cube([3,hh+EXPAND+1,12], center=true);
        }
    }
}

module RainBase(){
    difference(){
        union(){
//            rotate([180,0,0]) 
            difference() {
//                union() {
//                    cylinder(h=2*tloustka, r=diameter/2, center=false, $fn=200);
                    hull(){
                        translate([0,0,1]) cylinder(h=6, d2=47-6, d1=47+6, center=false, $fn=100);
                        translate([40,-6,1]) cylinder(h=6, d2=20-6, d1=20+6, center=false, $fn=100);
                        translate([-40,-6,1]) cylinder(h=6, d2=20-6, d1=20+6, center=false, $fn=100);
                            translate([0,0,-8.5]) cylinder(h=1, d=41+6, center=false, $fn=180);
//                            difference(){
//                                hull(){
//                                    translate([40,6,0]) sphere(d=20, center=false, $fn=100);
//                                    translate([-40,6,0]) sphere(d=20, center=false, $fn=100);
//                                }
//                                translate([0,0,-8]) cube([100,100,17], center=true);
//                            }
                        }
                        // bottom ring
                        translate([0,0,-8.6]) cylinder(h=5.6, d2=37.5+6, d1=40+6, center=false, $fn=80);
                        translate([0,0,-3.1]) cylinder(h=10.1-5, d2=37+5, d1=37.5, center=false, $fn=80);
                    }
                // bottom mout for rod
                translate([0, 0, 0]) scale([1.5,1,1]) cylinder(h=2.1, d1=19, d2=21, $fn=50, center=false);
                }
//                    }
                    // M3
                    translate([25, 13.7, 6.63]) rotate([135,0,0]){
                       translate([0,0,1.8]) cylinder(h=2, d1=3.2, d2=2.7, $fn=30, center=false);
                       translate([0,0,3.7]) cylinder(h=7, d=2.7, $fn=30, center=false);
                    }
                    translate([25,-19.75,6.63]) rotate([-135,0,0]){
                       translate([0,0,1.8]) cylinder(h=2, d1=3.2, d2=2.7, $fn=30, center=false);
                       translate([0,0,3.7]) cylinder(h=7, d=2.7, $fn=30, center=false);
                    }
                    translate([-25, 13.7, 6.63]) rotate([135,0,0]){
                       translate([0,0,1.8]) cylinder(h=2, d1=3.2, d2=2.7, $fn=30, center=false);
                       translate([0,0,3.7]) cylinder(h=7, d=2.7, $fn=30, center=false);
                    }
                    translate([-25,-19.75,6.63]) rotate([-135,0,0]){
                       translate([0,0,1.8]) cylinder(h=2, d1=3.2, d2=2.7, $fn=30, center=false);
                       translate([0,0,3.7]) cylinder(h=7, d=2.7, $fn=30, center=false);
                    }
                // -KEY
                translate([0,-RODshift,0]) rotate([90,0,90]) cylinder(h=12, d=6.2, $fn=4, center=true);
                // -ROD
                translate([0,-RODshift,-1]) cylinder(h=2*tloustka+0.2+5, d=6.2, $fn=30, center=false);
                    // NUT
                    translate([0,-RODshift,7-1.5]) cylinder(h=5, r=10/sin(60)/2, $fn=6, center=false);
                // -PCB
                translate([0,PCBshift+0.9, 3]) cube([PCBwidth+0.5, 3.5, 2*tloustka+7], center=true);
                    translate([0,-PCBshift-1.9, -8.5]) cube([7, 7.5, 3], center=true);
                // -h2o hole
                translate([41,-6,-3]) scale([1,2,1]) cylinder(h=10.1, d2=6, d1=2, $fn=30, center=false);
                translate([-41,-6,-3]) scale([1,2,1]) cylinder(h=10.1, d2=6, d1=2, $fn=30, center=false);
//             for (x = [0:30:360]){
//                rotate([0,0,x]) translate([0,-diameter/2-0.2+space,tloustka]) cube([5.4,tloustka,3*tloustka], center=true);
//             }
//             for (x = [0:30:360]){
//                rotate([0,0,x+8.8]) translate([0,-diameter/2-0.2+space,2.5*tloustka-1]) cube([5.4,tloustka,2], center=true);
//             }
             translate([0,-15.5,7.1]) rotate([0,0,0]) supp4swing(3, 0.2, 23.5);
//             translate([0,12,7.1]) rotate([0,0,0]) supp4swing(3, 0.2, 23.5);
//            }
        }        
        // -cut
//        if (CUT>0) {
//            translate([-70,0,0]) cube([140, 10*100, 200], center=true);
//        }
    
}

module swing0(XX, YY, ROT){
    rotate([0,ROT,0])
    difference(){
        union(){
            difference(){
                union(){
                    translate([0,0,-5]) hull(){
                        translate([-XX/2,0,5]) rotate([0,90, 90]) cylinder(h=YY, d=6, $fn=50, center=true);
                        translate([XX/2,0,5]) rotate([0,90, 90]) cylinder(h=YY, d=6, $fn=50, center=true);
                        translate([0,0,16]) rotate([0,90, 90]) cylinder(h=YY, d=6, $fn=50, center=true);
                        translate([0,0,5-6]) rotate([0,90, 90]) cylinder(h=YY, d=6, $fn=50, center=true);
                    }
                }
                // L water tank
                translate([0,0,-5]) hull(){
                    translate([-XX/2-2,0,5+2]) rotate([0,90, 90]) cylinder(h=YY-2, d=5, $fn=50, center=true);
                    translate([-1.5,0,-1.5]) rotate([0,90, 90]) cylinder(h=YY-2, d=2, $fn=50, center=true);
                    translate([-1.5,0,20]) rotate([0,90, 90]) cylinder(h=YY-2, d=2, $fn=50, center=true);
                }
                // R water tank
                translate([0,0,-5]) hull(){
                    translate([XX/2+2,0,5+2]) rotate([0,90, 90]) cylinder(h=YY-2, d=5, $fn=50, center=true);
                    translate([1.5,0,-1.5]) rotate([0,90, 90]) cylinder(h=YY-2, d=2, $fn=50, center=true);
                    translate([1.5,0,20]) rotate([0,90, 90]) cylinder(h=YY-2, d=2, $fn=50, center=true);
                }
            }
            hull(){
                // M3 outside
                rotate([0,90, 90]) cylinder(h=YY, d=6, $fn=50, center=true);
                // magnet outside
                translate([XX/7,0,-5.0]) rotate([0,78,0]) cube([MagnetX+2,YY,MagnetY+2], center=true); 
                translate([-XX/7,0,-5.0]) rotate([0,78+25,0]) cube([MagnetX+2,YY,MagnetY+2], center=true); 
            }
        }
        // -M3 center
        translate([0,-YY/4,0]) rotate([0,90, 90]) cylinder(h=YY/2+0.2, d2=2.5, d1=3, $fn=50, center=true);
        translate([0,YY/4,0]) rotate([0,90, 90]) cylinder(h=YY/2+0.2, d1=2.5, d2=3, $fn=50, center=true);
        // -magnet
        color([1,0,0]) translate([XX/7,-YY/2+6.4,-5.0]) rotate([0,78,0]) cube([MagnetX,12+1,MagnetY], center=true); 
        color([1,0,0]) translate([-XX/7,-YY/2+6.4,-5.0]) rotate([0,78+24,0]) cube([MagnetX,12+1,MagnetY], center=true); 
        // - water out
        translate([XX/3,0,0]) rotate([0,89, 0]) cylinder(h=XX/4, d2=4, d1=8, $fn=50, center=false);
        translate([-XX/3,0,0]) rotate([0,-89, 0]) cylinder(h=XX/4, d2=4, d1=8, $fn=50, center=false);

    }
    // PCB
//    translate([0,PCBshift, -6]) color([0,1,0]) cube([4, 1.6, 25], center=true);
//    translate([0,PCBshift, -11]) color([0,1,0]) cube([14, 1.6, 5], center=true);
}

module swing3(XX, YY, ROT){
    rotate([0,28,0])
    difference(){
//        translate([0,0,10]) scale([1.5,1,1]) sphere(d=34, $fn=300);
//        translate([0,0,10]) scale([1.5,1,1]) sphere(d=34-2, $fn=300);
//        translate([-30,-30,5]) cube([60,60,60]);
            union(){
                translate([0,10,-13]) rotate([90,0, 0]) cylinder(h=8, d=7, $fn=30, center=true);
                hull(){
                    translate([0,0,14]) rotate([90,0, 0]) cylinder(h=16*2, d=2, $fn=30, center=true);
                    translate([0,0,-2]) rotate([90,0, 0]) cylinder(h=16*2, d=2, $fn=30, center=true);

                    translate([30,0,-2]) rotate([90,0, 0]) cylinder(h=16, d=2, $fn=30, center=true);
                    translate([-30,0,-2]) rotate([90,0, 0]) cylinder(h=16, d=2, $fn=30, center=true);
                }
            }        
    }
}



module swing2(XX, YY, ROT){
    rotate([0,-28,0])
    difference(){
//        translate([0,0,10]) scale([1.5,1,1]) sphere(d=34, $fn=300);
//        translate([0,0,10]) scale([1.5,1,1]) sphere(d=34-2, $fn=300);
//        translate([-30,-30,5]) cube([60,60,60]);
            union(){
                hull(){
                    translate([0,0,0]) rotate([-90,0, 0]) cylinder(h=10, d=6, $fn=30, center=false);
                    translate([3,0,-12]) rotate([-90,0, 0]) cylinder(h=8, d=6, $fn=30, center=false);
                }
                hull(){
                    translate([-35,0,2]) rotate([90,0, 0]) cylinder(h=8, d=2, $fn=30, center=false);
                    translate([0,0,0]) rotate([90,0, 0]) cylinder(h=10, d=6, $fn=30, center=false);
                    translate([3,0,-12]) rotate([90,0, 0]) cylinder(h=8, d=6, $fn=30, center=false);
                }
                hull(){
                    translate([-35,0,2]) rotate([90,0, 0]) cylinder(h=8, d=2, $fn=30, center=false);
                    translate([-6,0,2]) rotate([90,0, 0]) cylinder(h=20, d=2, $fn=30, center=false);
                   # translate([-13,0,-7.358]) rotate([90,0, 0]) cylinder(h=20-5, d=2, $fn=30, center=false);
                }
            }
        
    }
}



module swing1(XX, YY, ROT){
    rotate([0,ROT,0])
    difference(){
        union(){
            difference(){
                union(){
                    translate([0,0,-5]) hull(){
                        translate([-XX/2,0,5]) rotate([0,90, 90]) cylinder(h=YY, d=6, $fn=50, center=true);
                        translate([XX/2,0,5]) rotate([0,90, 90]) cylinder(h=YY, d=6, $fn=50, center=true);
                        translate([0,0,16]) rotate([0,90, 90]) cylinder(h=YY, d=6, $fn=50, center=true);
                        translate([0,0,5-6]) rotate([0,90, 90]) cylinder(h=YY, d=6, $fn=50, center=true);
                    }
                }
                // L water tank
                translate([0,0,-5]) hull(){
                    translate([-XX/2-2,0,5+2]) rotate([0,90, 90]) cylinder(h=YY-2, d=5, $fn=50, center=true);
                    translate([-11,0,1.5]) rotate([0,90, 90]) cylinder(h=YY-2, d=2, $fn=50, center=true);
                    translate([-2.5,0,20]) rotate([0,90, 90]) cylinder(h=YY-2, d=2, $fn=50, center=true);
                }
                // R water tank
                translate([0,0,-5]) hull(){
                    translate([XX/2+2,0,5+2]) rotate([0,90, 90]) cylinder(h=YY-2, d=5, $fn=50, center=true);
                    translate([11,0,1.5]) rotate([0,90, 90]) cylinder(h=YY-2, d=2, $fn=50, center=true);
                    translate([2.5,0,20]) rotate([0,90, 90]) cylinder(h=YY-2, d=2, $fn=50, center=true);
                }
            }
            hull(){
                // M3 outside
                rotate([0,90, 90]) cylinder(h=YY, d=6, $fn=50, center=true);
                // magnet outside
                translate([XX/7,0,-5.0]) rotate([0,78,0]) cube([MagnetX+2,YY,MagnetY+2], center=true); 
                translate([-XX/7,0,-5.0]) rotate([0,78+25,0]) cube([MagnetX+2,YY,MagnetY+2], center=true); 
            }
        }
        // -M3 center
        translate([0,-YY/4,0]) rotate([0,90, 90]) cylinder(h=YY/2+0.2, d2=2.5, d1=3, $fn=50, center=true);
        translate([0,YY/4,0]) rotate([0,90, 90]) cylinder(h=YY/2+0.2, d1=2.5, d2=3, $fn=50, center=true);
        // -magnet
        color([1,0,0]) translate([XX/7,-YY/2+6.4,-5.0]) rotate([0,78,0]) cube([MagnetX,12+1,MagnetY], center=true); 
        color([1,0,0]) translate([-XX/7,-YY/2+6.4,-5.0]) rotate([0,78+24,0]) cube([MagnetX,12+1,MagnetY], center=true); 
        // - water out
        translate([XX/3,0,0.5]) rotate([0,89, 0]) cylinder(h=XX/4, d2=4, d1=8, $fn=50, center=false);
        translate([-XX/3,0,0.5]) rotate([0,-89, 0]) cylinder(h=XX/4, d2=4, d1=8, $fn=50, center=false);
        // bottom cut
        translate([0,-YY/2-0.1,5+5.6]) rotate([0,90, 90]) cylinder(h=3.5, d=5.8, $fn=30, center=false);
            translate([0,-YY/2-0.1,5+5.6]) rotate([0,90, 90]) cylinder(h=YY+2, d1=3, d2=2.8, $fn=30, center=false);
        translate([0,-YY/2-0.1,5]) rotate([0,90, 90]) cylinder(h=3.5, d=5.8, $fn=30, center=false);
            translate([0,-YY/2-0.1,5]) rotate([0,90, 90]) cylinder(h=YY+2, d1=3, d2=2.8, $fn=30, center=false);

        translate([0,0,-9]) rotate([0,90, 90]) rotate([0,0,60]) cylinder(h=YY+2, d=13, $fn=3, center=true);

    }
    // PCB
//    translate([0,PCBshift, -6]) color([0,1,0]) cube([4, 1.6, 25], center=true);
//    translate([0,PCBshift, -11]) color([0,1,0]) cube([14, 1.6, 5], center=true);
}


module swing(XX, YY, ROT){
    rotate([0,ROT,0])
    difference(){
        union(){
            difference(){
                union(){
                    hull(){
                        translate([-XX/2,0,0]) rotate([0,90, 90]) cylinder(h=YY, d=6, $fn=50, center=true);
                        translate([XX/2,0,0]) rotate([0,90, 90]) cylinder(h=YY, d=6, $fn=50, center=true);
                        translate([0,0,11+0]) rotate([0,90, 90]) cylinder(h=YY, d=6, $fn=50, center=true);
//                        translate([0,0,-6]) rotate([0,90, 90]) cylinder(h=YY, d=6, $fn=50, center=true);
                    }
                }
                // L water tank
                translate([0,0,-5]) hull(){
                    translate([-XX/2-2,0,5+2]) rotate([0,90, 90]) cylinder(h=YY-2, d=5, $fn=50, center=true);
                    translate([-11,0,7.5]) rotate([0,90, 90]) cylinder(h=YY-2, d=2, $fn=50, center=true);
                    translate([-2.5,0,26]) rotate([0,90, 90]) cylinder(h=YY-2, d=2, $fn=50, center=true);
                }
                // R water tank
                translate([0,0,-5]) hull(){
                    translate([XX/2+2,0,5+2]) rotate([0,90, 90]) cylinder(h=YY-2, d=5, $fn=50, center=true);
                    translate([11,0,7.5]) rotate([0,90, 90]) cylinder(h=YY-2, d=2, $fn=50, center=true);
                    translate([2.5,0,26]) rotate([0,90, 90]) cylinder(h=YY-2, d=2, $fn=50, center=true);
                }
            }
            hull(){
                // M3 outside
                rotate([0,90, 90]) cylinder(h=YY, d=6, $fn=50, center=true);
                // magnet outside
                translate([XX/7,0,-0.8]) rotate([0,0,0]) cube([MagnetY+2,YY,MagnetX+2], center=true); 
                translate([-XX/7,0,-0.8]) rotate([0,0,0]) cube([MagnetY+2,YY,MagnetX+2], center=true); 
            }
        }
        // -M3 center
        translate([0,-YY/4,0]) rotate([0,90, 90]) cylinder(h=YY/2+0.2, d2=2.5, d1=3, $fn=50, center=true);
        translate([0,YY/4,0]) rotate([0,90, 90]) cylinder(h=YY/2+0.2, d1=2.5, d2=3, $fn=50, center=true);
        // -magnet
        color([1,0,0]) translate([XX/7,-YY/2+6.4,-0.5]) cube([MagnetY,12+1,MagnetX], center=true); 
        color([1,0,0]) translate([-XX/7,-YY/2+6.4,-0.5]) cube([MagnetY,12+1,MagnetX], center=true); 
        // - water out
        translate([XX/3,0,2]) rotate([0,92, 0]) cylinder(h=XX/4, d2=6, d1=2, $fn=50, center=false);
        translate([-XX/3,0,2]) rotate([0,-92, 0]) cylinder(h=XX/4, d2=7, d1=2, $fn=50, center=false);
        // bottom cut
        translate([0,-YY/2-0.1,5+11.6]) rotate([0,90, 90]) cylinder(h=3.5, d=5.8, $fn=30, center=false);
            translate([0,-YY/2-0.1,5+11.6]) rotate([0,90, 90]) cylinder(h=YY+2, d1=3, d2=2.8, $fn=30, center=false);
        translate([0,-YY/2-0.1,11]) rotate([0,90, 90]) cylinder(h=3.5, d=5.8, $fn=30, center=false);
            translate([0,-YY/2-0.1,11]) rotate([0,90, 90]) cylinder(h=YY+2, d1=3, d2=2.8, $fn=30, center=false);


    }
    // PCB
//    translate([0,PCBshift, -6]) color([0,1,0]) cube([4, 1.6, 25], center=true);
//    translate([0,PCBshift, -11]) color([0,1,0]) cube([14, 1.6, 5], center=true);
}




module Bottom(DIA, ZZ){
    difference(){
        union(){
            cylinder(h=0.11*DIA, d2=0.86*DIA, d1=0.97*DIA, $fn=200);
            translate([0,0,-ZZ+4]) cylinder(h=ZZ-3.9, d=0.97*DIA, $fn=200);
        }
        cylinder(h=0.11*DIA+0.1, d2=0.86*DIA-4, d1=0.97*DIA-4, $fn=200);
        translate([0,0,-ZZ+4+2]) cylinder(h=ZZ-3.9-2, d=0.97*DIA-4, $fn=200);
        // Half
        translate([0,-20,0]) cube([10*DIA, 40+4, 3*DIA], center=true);

        // -RQ
       translate([16, 16, -ZZ+3]) rotate([0,0,20]) scale([1.3, 1.3, 1]) rq(2);

        // -ETH
        translate([-4.2, 22+1,-ZZ+5]) cube([16.5+0.5, 14+0.5, 4], center=true);

        // -cable
        hull(){
            translate([DIA/2-7, 0, -76-1]) cylinder(h=4, d=4, $fn=50);
            translate([DIA/2-7, 4, -76-1]) cylinder(h=4, d=4, $fn=50);
        }
        
        // -cut
        if (CUT>0) {
            translate([-20,0,0]) cube([40, 10*DIA, 3*DIA], center=true);
        }
    }
    
    // KeyDoor
    translate([80/2-5,-5,-ZZ/2+2]) cylinder(h=ZZ-4, d=5.5, $fn=30, center=true);
    hull(){
        translate([80/2-5,-5,-ZZ/2+2]) cylinder(h=ZZ-4, d=2, $fn=30, center=true);
        translate([80/2-5,2,-ZZ/2+2]) cylinder(h=ZZ-4, d=2, $fn=30, center=true);
    }
    translate([-80/2+5,-5,-ZZ/2+2]) cylinder(h=ZZ-4, d=5.5, $fn=30, center=true);
    hull(){
        translate([-80/2+5,-5,-ZZ/2+2]) cylinder(h=ZZ-4, d=2, $fn=30, center=true);
        translate([-80/2+5,2,-ZZ/2+2]) cylinder(h=ZZ-4, d=2, $fn=30, center=true);
    }
    // Mount clip
    difference(){
        hull(){
            translate([DIA/4+1.2,-5, -76]) cylinder(h=2, d=14, $fn=50);
            translate([DIA/4+1.2,5, -76]) scale([1,0.1,1]) cylinder(h=2, d=14, $fn=50);
        }
        translate([DIA/4+1.2,-5, -75]) cylinder(h=4, d=3.2, $fn=50, center=true);
    }
    // ETH ring
    difference(){
        translate([-4.2, 22+1, -ZZ+7.5]) cube([16.5+0.5+3, 14+0.5+3, 7], center=true);
        translate([-4.2, 22+1, -ZZ+7.5]) cube([16.5+0.5, 14+0.5, 7+2], center=true);
    }
    // door for wire
    translate([DIA/2-7, 4, -76]) cylinder(h=2, d=3, $fn=50);
    translate([DIA/2-7, 3, -76+1]) cube([3,2,2], center=true);
    translate([DIA/2-7, 4, -76+1]) cube([4,4,0.4], center=true);

}

module SunGuard(DIA, WINDOW, ZZ){
    difference(){
        union(){    // CUT
            difference(){
                union(){
                    difference(){
                        union(){
                            if (WINDOW==1) {
                                // OUT 1
                                cylinder(h=0.2*DIA, d2=0.73*DIA, d1=DIA, $fn=200);
                                difference(){
                                    translate([0,0,-ZZ+3.9]) cylinder(h=ZZ-3.8, d=DIA, $fn=200);
                                    translate([0,PCBshift-0.8+DIA/2, -ZZ/2]) cube([DIA+2, DIA, ZZ], center=true);
                                }
                            }else if (WINDOW==4) {
                                hull(){
                                    translate([0,0,0.2*DIA-5]) rotate_extrude(convexity = 1, $fn = 250) translate([0.73*DIA/2-5, 0, 0]) circle(r = 5, $fn = 50);
                                    cylinder(h=0.2, d2=0.73*DIA, d1=DIA, $fn=200);
                                }
                            }else{
                                    cylinder(h=0.2*DIA, d2=0.73*DIA, d1=DIA, $fn=200);
                            }
                        }
                        if (WINDOW==1) {
                            // -IN 1
                            difference(){
                                translate([0,0,-1.5+1.4]) cylinder(h=0.2*DIA-1.5, d2=0.73*DIA-2, d1=DIA-3, $fn=200);
                                translate([0,PCBshift-0.8-DIA/2, 0.1*DIA]) cube([DIA+2, DIA, 0.3*DIA], center=true);
                            }
                        }else if (WINDOW==4) {
                            hull(){
                                translate([0,0,0.2*DIA-5-1.5]) rotate_extrude(convexity = 1, $fn = 50) translate([0.73*DIA/2-5, 0, 0]) circle(r = 5, $fn = 50);
                                translate([0,0,-1.5]) cylinder(h=0.2, d2=0.73*DIA-2, d1=DIA-2, $fn=200);
                            }        
                        }else{
                                translate([0,0,-1.5]) cylinder(h=0.2*DIA, d2=0.73*DIA-2, d1=DIA-2, $fn=200);


//                            if (WINDOW==1) {
//                                // -RQ
//                                translate([20, 0, 0.2*DIA-0.5]) scale([1.2, 1.2, 1]) rq(2);
//                            }
                        }
                    }
                    if (WINDOW==1) {
//                        hull(){
//                            translate([7,0,-ZZ+4]) cylinder(h=ZZ+0.2*DIA-10+6, d=20, $fn=30);
//                            translate([-7,0,-ZZ+4]) cylinder(h=ZZ+0.2*DIA-10+6, d=20, $fn=30);
//                        }
                    }else{
                        if (WINDOW==4) {
                            translate([0,0,0.1*DIA+7]) cylinder(h=0.1*DIA+4, d1=20+8, d2=20, $fn=30);
                            translate([0,0,0.1*DIA]) cylinder(h=0.1*DIA+11, d1=20, d2=20, $fn=30);
                            // KEY
                            difference(){
                                translate([0,-RODshift,0.1*DIA+18.9]) rotate([90,0,90]) cylinder(h=20, d=6.2, $fn=4, center=true);
                                difference(){
                                    translate([0,0,0.1*DIA+18.9]) cylinder(h=7+1, r=10+5, $fn=30, center=true);                    
                                    translate([0,0,0.1*DIA+18.9]) cylinder(h=7, r=10, $fn=30, center=true);
                                }
                            }
                            translate([0,-RODshift,0.1*DIA+0.1]) rotate([90,0,90]) cylinder(h=14, d=6.2, $fn=4, center=true);
                        }else{
                            translate([0,0,0.1*DIA]) cylinder(h=0.1*DIA, d=20, $fn=30);
                            // KEY
                            translate([0,-RODshift,0.1*DIA+0.1]) rotate([90,0,90]) cylinder(h=14, d=6.2, $fn=4, center=true);
                        }
                    }

                }
                if (WINDOW<4) {
                    // -KEY
                    translate([0,-RODshift,0.2*DIA]) rotate([90,0,90]) cylinder(h=15, d=6.2, $fn=4, center=true);
                }
                // -PCB
                if (WINDOW==2) {
                    translate([0,PCBshift+7, 0.15*DIA]) cube([13+20, 1.6+14, 0.2*DIA], center=true);
                    translate([-8.8, 2.5, 0.15*DIA]) cube([20, 4, 0.2*DIA], center=true);
                }else if (WINDOW==1) {
                    translate([0,PCBshift+5, -ZZ/2+0.1*DIA-0.76]) cube([13+7, 1.6+10, ZZ+0.2*DIA-1.5], center=true);
                    translate([0,PCBshift+1.5, -ZZ/2]) cube([32, 1.6+3, DIA+ZZ], center=true);
                    //-B connector
                    translate([0, +0.8+PCBshift+2, 25]) cube([8.5, 4+3, DIA+ZZ], center=true);
                    // M3
                    translate([12,PCBshift-0.79,3+1+2]) M(3, 10);
                    translate([-12,PCBshift-0.79,3+1+2]) M(3, 10);
                    translate([12,PCBshift-0.79, -65+1+2]) M(3, 10);
                    translate([-12,PCBshift-0.79, -65+1+2]) M(3, 10);
                    // Bottom mount
                    translate([DIA/4,-5, -77]) rotate([-90,0,0]) M(3, 15);
//                    translate([DIA/4,-5, -76]) cube([15,15,4.5], center=true);
                    hull(){
                        translate([DIA/4,-5, -76]) cylinder(h=4, d=15, $fn=50, center=true);
                        translate([DIA/4,5, -76]) cylinder(h=4, d=15, $fn=50, center=true);
                    }

                    // MOUNT
                    if(MountDia<50){
                        translate([0,-DIA/2,-ZZ/2]) cylinder(h=ZZ, d=MountDia, $fn=MountDiaFn, center=true);
                        translate([0,-DIA/2,-ZZ/2]) rotate([0,90,0]) rotate([0,0,45]) cylinder(h=ZZ, d=MountDia, $fn=MountDiaFn, center=true);
                    }else{
                        translate([0,-DIA/2-(MountDia-50),-ZZ/2]) cylinder(h=ZZ, d=MountDia, $fn=100, center=true);
                        translate([0,-DIA/2-(MountDia-50),-ZZ/2]) rotate([0,90,0]) rotate([0,0,45]) cylinder(h=ZZ, d=MountDia, $fn=MountDiaFn, center=true);
                    }
                    // MountBinder
                    if(MountDia<50){
                        translate([0,-DIA/2,-6]) Binder(MountDia);
                        translate([0,-DIA/2,-ZZ+10]) Binder(MountDia);
                        translate([MountDia/2+4,-DIA/2,-ZZ/2]) rotate([0,90,0]) Binder(MountDia);
                        translate([-MountDia/2-4,-DIA/2,-ZZ/2]) rotate([0,90,0]) Binder(MountDia);
                    }else{
                        translate([0,-DIA/2-(MountDia-50),-6]) Binder(MountDia);
                        translate([0,-DIA/2-(MountDia-50),-ZZ+10]) Binder(MountDia);
                        translate([MountDia/2+4-(MountDia-50),-DIA/2-(MountDia-50),-ZZ/2]) rotate([0,90,0]) Binder(MountDia);
                        translate([-MountDia/2-4+(MountDia-50),-DIA/2-(MountDia-50),-ZZ/2]) rotate([0,90,0]) Binder(MountDia);
                    }
                        translate([23, -1, -58]) Binder(1);

                    // space for pinheader
                    hull(){
                        translate([0, 2, -ZZ/2+6]) rotate([0,90,0]) cylinder(h=45, r=1.5, $fn=30, center=true);
                        translate([0, 2, -ZZ/2+9]) rotate([0,90,0]) cylinder(h=45, r=1.5, $fn=30, center=true);
                    }
                    translate([-12.7, 2, -ZZ/2+6]) cylinder(h=33, d=6, $fn=30, center=true);
//                        translate([-12.7,  8, -ZZ/2+7.5]) cylinder(h=6, d=22, $fn=60, center=true);
                    translate([12.7, 2, -ZZ/2+6-3]) cylinder(h=33+6, d=6, $fn=30, center=true);
//                        translate([12.7,  8, -ZZ/2+7.5]) cylinder(h=6, d=22, $fn=60, center=true);
                    // KeyDoor
                    translate([DIA/2-5,-5,-ZZ/2]) cylinder(h=ZZ+5, d=6, $fn=30, center=true);
                    hull(){
                        translate([DIA/2-5,-5,-ZZ/2]) cylinder(h=ZZ+8, d=2.5, $fn=30, center=true);
                        translate([DIA/2-5,2,-ZZ/2]) cylinder(h=ZZ+8, d=2.5, $fn=30, center=true);
                    }
                    translate([-DIA/2+5,-5,-ZZ/2]) cylinder(h=ZZ+5, d=6, $fn=30, center=true);
                    hull(){
                        translate([-DIA/2+5,-5,-ZZ/2]) cylinder(h=ZZ+8, d=2.5, $fn=30, center=true);
                        translate([-DIA/2+5,2,-ZZ/2]) cylinder(h=ZZ+8, d=2.5, $fn=30, center=true);
                    }
                    translate([DIA/2, 2, -ZZ/2]) cylinder(h=ZZ, d=8, $fn=4, center=true);
                    translate([-DIA/2, 2, -ZZ/2]) cylinder(h=ZZ, d=8, $fn=4, center=true);
                    // N-S
                    translate([0,-15,-ZZ+4]) rotate([0,90,0]) cylinder(h=ZZ, d=5, $fn=4, center=true);
                    translate([-30,-10,-ZZ+5]) {rotate([180,0,90]){
                        linear_extrude(height = 2, center = false, convexity = 5, twist = 0, slices = 20, scale = 1.0) {
                        text(text =  "N", font = "Marsh:style=Stencil", halign="left", size=10);
                    }}}
                    translate([-ZZ/2+3.5,-13,-ZZ+17]) {rotate([-90,0,103]){
                        linear_extrude(height = 2, center = false, convexity = 5, twist = 0, slices = 20, scale = 1.0) {
                        text(text =  "N", font = "Marsh:style=Stencil", halign="left", size=10);
                    }}}
                    translate([28,-12,-ZZ+5]) {rotate([180,0,90]){
                        linear_extrude(height = 2, center = false, convexity = 5, twist = 0, slices = 20, scale = 1.0) {
                        text(text =  "S", font = "Marsh:style=Stencil", halign="left", size=5);
                    }}}
                    translate([ZZ/2-3.5,-14,-ZZ+6]) {rotate([90,0,77]){
                        linear_extrude(height = 2, center = false, convexity = 5, twist = 0, slices = 20, scale = 1.0) {
                        text(text =  "S", font = "Marsh:style=Stencil", halign="left", size=10);
                    }}}
                    
                }else{
                    translate([0,PCBshift+1.5, -ZZ/2]) cube([13+4, 1.6+3, DIA+ZZ], center=true);
                    translate([0,PCBshift+1.5, 0]) cube([13+14, 1.6+3, 0.4*DIA-3], center=true);
                    //-B connector
                    translate([0, +0.8+PCBshift+2, 25]) cube([8.5, 7, DIA+ZZ], center=true);
                }
                // -ROD
                translate([0,-RODshift, -ZZ-1]) cylinder(h=ZZ+0.2*DIA+16, d=6.2, $fn=30, center=false);
                if (WINDOW==2) {
                    for(ROT = [0:60:360]){
                        rotate([0,0,ROT]) translate([0,0.23*DIA,0.2*DIA-3]) cylinder(h=4, d=0.20*DIA, $fn=50, center=false);
                    }
                }
                if (WINDOW<4) {
                    for(ROT = [30:60:360]){
                        rotate([0,0,ROT]) translate([0,0.29*DIA,0.2*DIA-1.51]) cylinder(h=1.52, d1=2, d2=2.5, $fn=50, center=false);
                    }
                }
            }
                if (WINDOW>1) {
                    for(ROT = [30:60:360]){
                        rotate([0,0,ROT]) translate([0,0.29*DIA,0.1*DIA]) cylinder(h=0.1*DIA-1.5, d1=3, d2=6, $fn=50, center=false);
                        rotate([0,0,ROT]) translate([0,0.29*DIA,0.1*DIA-1]) cylinder(h=1.52, d1=2, d2=2.5, $fn=50, center=false);
                    }
                }
                if (WINDOW==1) {
                    for(ROT = [30:60:360]){
                        rotate([0,0,ROT]) translate([0,0.29*DIA,0.1*DIA+5.5]) cylinder(h=1, d1=3, d2=6, $fn=50, center=false);
                    }
//                    hull(){
//                        translate([0,-10,-ZZ+10]) cylinder(h=ZZ+0.2*DIA-10, d=4, $fn=30);
//                        translate([0,-0.35*DIA+2,-ZZ+10]) cylinder(h=ZZ+0.2*DIA-10, d=4, $fn=30);
//                    }
//                    hull(){
//                        translate([0,-0.5*DIA+2,-ZZ+10]) cylinder(h=ZZ-10, d=4, $fn=30);
//                        translate([0,-0.35*DIA+2,-ZZ+10]) cylinder(h=ZZ+0.2*DIA-10, d=4, $fn=30);
//                    }
                }
            }
        // -cut
        if (CUT>0) {
            translate([-20,0,0]) cube([40, 10*DIA, 3*DIA], center=true);
        }
    }
}

module Binder(DIA){
    difference(){
        cylinder(h=6, d=DIA+6+6, $fn=100, center=true);
        cylinder(h=8, d=DIA+6, $fn=100, center=true);
    }
}

module WindDir(ZZ){
    difference(){
        union(){
            rotate([-5,0,0]) translate([0, 25, 25]) rotate([180,0,30]) cylinder(h=18, d1=8.0, d2=7.5,$fn=3, center=false); 
                translate([0, 27.5 ,ZZ-19]) cube([10,9,2], center=true);
            translate([0,30,ZZ/2-4]) rotate([180,0,0]) cube([1.6,11,12], center=true);
            translate([0,30+2,4.6]) rotate([45,0,0]) cube([10,2,2], center=true);
            hull(){
                translate([0,30+2,ZZ/2-15/2]) cube([10,2,15+4], center=true);
                translate([0, 4*ZZ ,ZZ/2+2]) rotate([180,0,0]) cylinder(h=2*ZZ+6, d=1, $fn=50, center=false);
            }
        }
        translate([0,2*ZZ,ZZ-8]) cube([12,200,20], center=true);
            translate([0, 27.5-2.5 ,ZZ-19+1.3]) rotate([30,0,0]) cube([12,6,2], center=true);
        translate([0,67.5-3,-37]) rotate([0,90,0]) cylinder(h=25, d=60, $fn=30, center=true);                
        translate([0,2*ZZ,-ZZ-20]) cube([12,200,20], center=true);

    }
}

module Out(ZZ, MINUS, WALL, TYPE){  // [1]-anemo, [2]-direction
    difference(){
        union(){
            if (TYPE==1) {
            translate([0,0,2.5+WALL]) rotate([0,0,0]) cylinder(h=ZZ/2-(2.5+WALL), r2=11, r1=12+WALL, $fn=250, center=false);
            translate([0,0,2.5+WALL+ZZ/2-(2.5+WALL)-1]) rotate([0,0,0]) cylinder(h=7, r=11.5, $fn=250, center=false);
            translate([0,0,2.5+WALL]) rotate([0,0,0]) cylinder(h=ZZ/2-(2.5+WALL), r2=11, r1=12+WALL+6, $fn=250, center=false);
                translate([0,0,2.5+WALL]) rotate([180,0,0]) cylinder(h=10, r=12+WALL+6, $fn=250, center=false);                
            translate([0,0,2.5+WALL-10]) rotate([180,0,0]) cylinder(h=ZZ-(ZZ/2-2.5-WALL)-10, r1=12+WALL+6, r2=12+WALL+4, $fn=250, center=false);
            }
            if (TYPE==2) {
            translate([0,0,2.5+WALL]) rotate([0,0,0]) cylinder(h=ZZ/2-(2.5+WALL), r2=12, r1=12+WALL+1, $fn=250, center=false);
            translate([0,0,2.5+WALL+ZZ/2-(2.5+WALL)-3]) scale([1, 1.2, 1])  cylinder(h=6+3, r=12, $fn=250, center=false);
            translate([0,0,2.5+WALL]) rotate([180,0,0]) cylinder(h=ZZ-(ZZ/2-2.5-WALL)+9, r1=12+WALL+1, r2=12+WALL+3+3, $fn=250, center=false);
//                translate([0,0,2.5+WALL-24]) rotate([180,0,0]) cylinder(h=10, r1=12+WALL+3, r2=12+WALL+8, $fn=250, center=false);
                translate([0,0,-3]) hull(){
                    translate([0,30+3,ZZ/2-15/2]) rotate([180,0,0]) cube([10,2,15], center=true);
                    translate([0,0,ZZ/2]) rotate([180,0,0]) cylinder(h=15, d=22, $fn=50, center=false);
                    translate([0,-30,ZZ/2]) rotate([180,0,0]) cylinder(h=15, d=10, $fn=50, center=false);
                }
                // magnet dock
//                translate([0, -13.5, 26-10]) cylinder(h=10, d1=6.2+5, d2=6.2+1, $fn=30, center=false); 
                hull(){
                    translate([0,-13-2,26-1]) cube([15, 8+4, 2], center=true);
                    translate([0,-13-2,26-6-3]) cube([13.5, 8+4, 1], center=true);
                }
//                translate([0,-30,ZZ/2+6]) rotate([180,0,0]) cylinder(h=15+6, d=10, $fn=50, center=false);
                  // Shark
//                hull(){
//                    translate([0,25,ZZ/2+6]) rotate([180,0,0]) scale([1,0.3,1]) cylinder(h=15+6, d=6, $fn=50, center=false);
//                    translate([0, 3.5*ZZ ,ZZ/2+6]) rotate([180,0,0]) cylinder(h=1.8*ZZ+6, d=1, $fn=50, center=false);
//                }
            }
        }
        // inner radius
        translate([0,0,0]) cylinder(h=5.2, d=24, $fn=200, center=true);
        if (TYPE==1) {
            translate([0,0,0]) rotate([0,0,0]) cylinder(h=ZZ/2+7, r1=10, r2=10, $fn=500, center=false);
            translate([0,0,-2.5]) rotate([180,0,0]) cylinder(h=ZZ-(ZZ/2+2.5)+1, r2=12+4+MINUS, r1=12, $fn=500, center=false);
            // -MagnetX
            color(c=[1,0,0,1])  translate([-MagnetY/2, 10.5+0.8, 3.5]) cube([MagnetY,MagnetX,MagnetZ+5]); 
           // -anemo plug
             for(ROT = [-6:120:360]){
                rotate([0,0,ROT]) translate([0,16,0]) rotate([0,90,-10]) cylinder(h=25+6, d=7, $fn=4, center=true);                
                hull(){
                    rotate([0,0,ROT]) translate([0,16,0]) rotate([0,90,-10]) translate([0,0,-10.5]) cylinder(h=25, d=7, $fn=4, center=true);                
                    rotate([0,0,ROT]) translate([0,16,0]) rotate([0,90,-10]) translate([0,10,-10.5]) cylinder(h=25, d=7+15, $fn=4, center=true);                
                }
             }
        }
      if (TYPE==2) {
            translate([0,0,0]) rotate([0,0,0]) cylinder(h=ZZ/2+7, r1=10, r2=11, $fn=500, center=false);
            translate([0,0,-2.5]) rotate([180,0,0]) cylinder(h=ZZ-(ZZ/2+2.5)+1, r2=12+MINUS+2, r1=12, $fn=500, center=false);
                translate([0,0,2.5+WALL-24]) rotate([180,0,0]) cylinder(h=10.2, r1=12+MINUS+1.5, r2=12+WALL+6, $fn=250, center=false);
          // M6 balancing weights
            translate([0, -11-1, ZZ/2-10]) rotate([90,0,0]) cylinder(h=24, d=5.6, $fn=50, center=false);
                translate([0, -11-1-17.2, ZZ/2-10]) rotate([90,0,0]) cylinder(h=6, d2=6.2, d1=5.6, $fn=50, center=false);
        // -MagnetX
//        translate([0, -13.5, 26-10]) cylinder(h=11, d=6.2, $fn=30, center=false); 
        color(c=[1,0,0,1]) translate([0,-13-2,26-1]) cube([12.2, 6.2, 2.2+1.2+1], center=true);
          difference(){
            translate([0, 0, 26-9]) cylinder(h=10, d1=38+10, d2=34+10, $fn=30, center=false); 
            translate([0, 0, 26-9]) cylinder(h=10, d1=37+4, d2=33.5+6, $fn=130, center=false); 
          }
          
        // -mount for Shark
        rotate([-5,0,0]) translate([0, 25+3, 25-3]) rotate([180,0,30]) cylinder(h=20, d1=8.0, d2=8.0,$fn=3, center=false); 
        translate([0,30+3,ZZ/2-3]) rotate([180,0,0]) cube([2,8,20], center=true);
        // -RQ
       translate([-0.5, -23, 0.5]) rotate([0,0,0]) scale([1.0, 1.0, 1]) rq(2);
      }
        // -cut
        if (CUT>0) {
            translate([-20,0,0]) cube([40, 10*ZZ, ZZ+40], center=true);
        }
    }
//      if (TYPE==2) {
//          // magnet support
//            translate([-3.3,-13-2,26-1.6]) cube([0.5, 6.2, 2.2+1], center=true);
//            translate([0,-13-2,26-1.6]) cube([0.5, 6.2, 2.2+1], center=true);
//            translate([3.3,-13-2,26-1.6]) cube([0.5, 6.2, 2.2+1], center=true);
//      }

}


module In(ZZ, MINUS, PCBSHIFT, BT){
    difference(){
        union(){
            if(PCBSHIFT==0){
                translate([0,0,2.5]) cylinder(h=5+(ZZ/2-7.5)+9, r1=7.5+BT/1000 , r2=7.5-MINUS, $fn=200);
            }
            translate([0,0,2.5]) cylinder(h=5+(ZZ/2-7.5)+9-16, r1=7.5+BT/1000 , r2=7.5-MINUS-1, $fn=200);
                translate([0,0,2.5]) cylinder(h=5+(ZZ/2-7.5)+9, r1=7.5-MINUS-1 , r2=7.5-MINUS-1, $fn=200);

            translate([0,0,-2.5]) cylinder(h=5, r=7.5+BT/1000 , $fn=200);
            translate([0,0,-2.5]) rotate([180,0,0]) cylinder(h=5+(ZZ/2-7.5)-9, d=15+3, $fn=50);
            // KEY
            if(PCBSHIFT==0){
                difference(){
                    translate([0,-RODshift,ZZ-11.1]) rotate([90,0,90]) cylinder(h=16, d=6.2, $fn=4, center=true);
                    difference(){
                        translate([0,0,ZZ-11.1]) cylinder(h=7+1, r=7.5-MINUS+5, $fn=50, center=true);                    
                        translate([0,0,ZZ-11.1]) cylinder(h=7, r=7.5-MINUS, $fn=100, center=true);
                    }
                }
            }else{
                difference(){
                    hull(){
                        translate([0,-RODshift,ZZ-11.1+1.6]) rotate([90,0,90]) cylinder(h=16, d=6.2, $fn=4, center=true);
                        translate([0,-RODshift,ZZ-11.1]) rotate([90,0,90]) cylinder(h=16, d=6.2, $fn=4, center=true);
                    }
                    difference(){
                        translate([0,-RODshift,ZZ-11.1]) cylinder(h=15, r=7.5-MINUS+5, $fn=50, center=true);                    
                        hull(){
                            translate([2,-RODshift,ZZ-11.1]) cylinder(h=16, d=6, $fn=100, center=true);
                            translate([-2,-RODshift,ZZ-11.1]) cylinder(h=16, d=6, $fn=100, center=true);
                        }
                    }
                }
            }
        }
        if(PCBSHIFT>0){
            //-B connector
            translate([0, +0.8+PCBshift+1.5, 21+1]) cube([8+2,3.5+2,17], center=true);
//                translate([0, +0.8+PCBshift+0.5, 23]) cube([5,3.5+2,5], center=true);
        }
        
        // - SMD
        if(PCBSHIFT==0){
//            translate([6, PCBshift+0, 8]) cube([5,4,10], center=true);
//            translate([3, PCBshift, 8]) scale([1, 1, 1.6]) rotate([0,90,0]) cylinder(h=5, d=7, $fn=50, center=false);
            difference(){
                translate([-2, PCBshift+1, 8.5]) scale([1, 1, 1.6]) rotate([0,-90,0]) cylinder(h=6, d=7, $fn=50, center=false);
                translate([-2, PCBshift+0.7, 8.5]) rotate([90,0,0]) cylinder(h=6, d=17, $fn=50, center=false);
            }
  
        }
        // -PCB
        translate([0,PCBshift, 0]) cube([13+5, 1.6, ZZ+21], center=true);
        // Sign
        translate([-3.5, 1.7, 3]) {rotate([0,-90,-90]){
            linear_extrude(height = 2, center = false, convexity = 5, twist = 0, slices = 20, scale = 1.0) {
            text(text =  str(BT,""), font = "Marsh:style=Stencil", halign="center", size=7);
        }}}
        // -ROD
        translate([0,-RODshift,0]) cylinder(h=ZZ+35, d=6.2, $fn=30, center=true);
        // -KEY
        translate([0,-RODshift,-11]) rotate([90,0,90]) cylinder(h=20, d=6.2, $fn=4, center=true);
        // -cut
        if (CUT>0) {
            translate([-20,0,0]) cube([40, 40, ZZ+40], center=true);
        }
        // -M2
        translate([5,PCBshift-0.8, -6.85]) M(2, 10);
        translate([-5,PCBshift-0.8, -6.85]) M(2, 10);

    }
}

module AnemoThird(DIA, WALL){
    difference(){
        union(){
            translate([0, 1.5*DIA, 0]) sphere(r = DIA/2, $fn=DIA*3);
//            translate([WALL*4-0.163*WALL, 20, 0]) scale([1, 1, 0.5]) rotate([-90,0,0]) cylinder(h=DIA*1.5-33, d=WALL*8, $fn=11);
            hull(){
                if(Strong==0){
                    translate([WALL*4-0.163*WALL, DIA+3, 0]) scale([1, 1, 0.5]) rotate([-90,0,0]) cylinder(h=1, d=WALL*8, $fn=11);
                }else{
                    translate([WALL*7.2, DIA+6, 0]) scale([1, 1, 0.25]) rotate([-90,0,0]) cylinder(h=1, d=WALL*15, $fn=11);
                }
                rotate([0,0,-6]) translate([0,16,0]) rotate([0,90,-10]) cylinder(h=2, d=7, $fn=4, center=false);                
                rotate([0,0,0]) translate([0,16.25,0]) rotate([0,90,0]) cylinder(h=2, d=7, $fn=4, center=false);
            }
            // shark
//            hull(){
//                translate([0.5*DIA, 1.5*DIA, 0]) cylinder(h=1, d=0.1, center=true, $fn=DIA*3);
//                translate([0.5, 1.5*DIA, 0]) cylinder(h=1, d=0.1, center=true, $fn=DIA*3);
//                translate([0.5, 15, 0]) cylinder(h=1, d=1, center=true, $fn=DIA*3);
//            }
            difference(){
                rotate([0,0,-6]) translate([0,16,0]) rotate([0,90,-10]) cylinder(h=15, d1=7, d2=6.5, $fn=4, center=false);                
                rotate([0,0,-6]) translate([13,15,0]) rotate([0,90,-10+55]) cylinder(h=5, d=10, $fn=30, center=false);                
            }
        }
        difference(){
            if(Strong==0){
                translate([0, 1.5*DIA, 0]) scale([0.96, 1, 1]) sphere(r = DIA/2-WALL, $fn=DIA*3);
            }else{
                translate([0, 1.5*DIA+1, 0]) scale([1, 1, 1]) sphere(r = DIA/2-WALL-1, $fn=DIA*3);
            }
            difference(){
//                translate([DIA/2, 1.5*DIA, 0]) cube([DIA/8, DIA+1, DIA+1], center=true);
//                translate([DIA/3-0.1, 1.5*DIA, 0]) rotate([0,90,0]) cylinder(h=DIA/6-WALL, d1=DIA/1.44, d2=0.1, $fn=200, center=false);
            }
        }
        translate([-DIA/2, 1.5*DIA, 0]) cube([DIA, DIA+1, DIA+1], center=true);
//        translate([0, 0, -5]) cylinder(h=10, d=25, $fn=200);
    }
}

fudge = 0.1;
module rq(h)
{
  scale([25.4/90, -25.4/90, 1]) union()
  {
    linear_extrude(height=h)
      polygon([[-7.281275,-22.968725],[-9.218775,-22.687475],[-11.086650,-21.758780],[-12.571819,-20.311942],[-13.552341,-18.466933],[-13.906275,-16.343725],[-13.875075,8.125025],[-11.843825,3.968775],[-11.562525,3.562525],[-11.562525,-16.343725],[-11.369259,-17.631210],[-10.828399,-18.759736],[-9.998352,-19.672444],[-8.937525,-20.312475],[-7.281275,-20.656225],[12.093725,-20.625025],[13.773681,-20.293083],[15.133026,-19.383076],[16.043033,-18.023731],[16.374975,-16.343775],[16.406175,16.343725],[16.074232,18.023677],[15.164223,19.383023],[13.804877,20.293032],[12.124925,20.624975],[1.062425,20.624975],[-0.093825,21.218725],[-0.156325,21.624975],[-0.425195,22.379917],[-0.968825,22.968725],[12.093675,22.937525],[14.667363,22.420090],[16.773759,21.008226],[18.196363,18.912575],[18.718675,16.343775],[18.718675,-16.343725],[18.201243,-18.917406],[16.789376,-21.023805],[14.693722,-22.446414],[12.124925,-22.968725],[-7.281325,-22.968725]]);
    linear_extrude(height=h)
      polygon([[10.968725,-16.437475],[-6.156275,-16.406275],[-6.875025,-16.281275],[-7.685400,-15.619512],[-8.000025,-14.593775],[-8.000025,3.062475],[-7.562525,3.156175],[-6.750756,3.499552],[-6.165395,4.158513],[-5.920108,5.005101],[-6.062525,5.874925],[-6.656275,7.374925],[-5.656275,7.593675],[-4.837019,7.995139],[-4.281275,8.718675],[-3.656275,8.843675],[-2.531275,9.062425],[-1.699656,9.444093],[-1.125025,10.156175],[-1.031325,10.187375],[-0.937625,10.218575],[-0.343875,10.343575],[0.486230,10.772203],[1.031125,11.531075],[1.437375,11.749825],[10.968725,11.781025],[12.272699,11.225624],[12.812475,9.906025],[12.812475,-14.593975],[12.272699,-15.897949],[10.968725,-16.437725]]);
    linear_extrude(height=h)
      polygon([[-9.906275,4.781275],[-9.906275,4.812475],[-9.999975,4.874975],[-10.031175,4.843775],[-12.249925,9.375025],[-14.468675,14.000025],[-15.062425,13.812525],[-15.656175,13.687525],[-15.093675,12.656275],[-14.562425,11.625025],[-15.218675,11.468775],[-15.843675,11.312525],[-15.656175,10.968775],[-15.437425,10.625025],[-16.656175,10.375025],[-17.843675,10.062525],[-18.281175,10.750025],[-18.718675,11.406275],[-18.093675,11.562525],[-17.468675,11.718775],[-17.687425,12.062525],[-17.874925,12.375025],[-17.249925,12.531275],[-16.624925,12.687525],[-16.999925,13.375025],[-17.374925,14.093775],[-16.749925,14.250025],[-16.093675,14.437525],[-16.437425,15.093775],[-16.812425,15.781275],[-16.156175,15.968775],[-15.468675,16.125025],[-15.812425,16.875025],[-16.156175,17.593775],[-15.468675,17.781275],[-14.781175,17.968775],[-15.093675,18.718775],[-15.374925,19.437525],[-9.593675,21.031275],[-3.749925,22.593775],[-3.624925,21.781275],[-3.437425,20.968775],[-2.781175,21.156275],[-2.124925,21.312525],[-1.937425,20.093775],[-1.749925,18.906275],[-1.062425,19.062525],[-0.343675,19.218775],[-0.031175,16.312525],[0.281325,13.375025],[-0.312425,13.250025],[-0.906175,13.093775],[-0.843675,12.687525],[-0.781175,12.312525],[-1.374925,12.156275],[-1.968675,12.062575],[-2.156175,13.281325],[-2.343675,14.500075],[-2.968675,14.343825],[-3.593675,14.218825],[-3.281175,12.593825],[-2.937425,11.031325],[-4.062425,10.781325],[-5.218675,10.531325],[-5.531175,11.687575],[-5.812425,12.843825],[-6.406175,12.687575],[-6.999925,12.562575],[-6.562425,11.062575],[-6.093775,9.531325],[-7.187525,9.281325],[-8.312525,9.062575],[-8.875025,10.531325],[-9.406275,12.031325],[-10.000025,11.875075],[-10.625025,11.750075],[-9.281275,8.437575],[-7.937525,5.125075],[-8.906275,4.937575],[-9.906275,4.781325]]);
    linear_extrude(height=h)
      polygon([[2.124975,14.625025],[1.937475,16.500025],[1.812475,17.812525],[3.218725,17.781325],[3.929543,17.492142],[4.218725,16.781325],[4.218725,15.656325],[3.913914,14.929882],[3.187475,14.625075],[2.124975,14.625075]]);
    linear_extrude(height=h)
      polygon([[9.531225,14.625025],[9.124975,14.718725],[8.675304,15.071853],[8.499975,15.624975],[8.531175,16.781225],[8.820361,17.492043],[9.531175,17.781225],[11.124925,17.812425],[11.835746,17.507614],[12.124925,16.781175],[12.124925,15.624925],[11.835743,14.914111],[11.124925,14.624925],[9.531175,14.624925]]);
  }
}

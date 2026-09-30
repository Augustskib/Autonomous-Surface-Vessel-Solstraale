#pragma once
struct ModulinoType {void begin(){}};
inline ModulinoType Modulino;
struct ModulinoMovement {
 void begin(){} void update(){}
 float getX(){return 0;} float getY(){return 0;} float getZ(){return 1;}
 float getRoll(){return 0;} float getPitch(){return 0;} float getYaw(){return 0;}
};

#ifndef COORDINATE_H
#define COORDINATE_H

public struct POINT2D {
    float x;
    float y;
};

POINT2D polar_to_cartesian(float r, float theta);

#endif // COORDINATE_H
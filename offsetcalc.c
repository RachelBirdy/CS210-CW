#include <stdio.h>
#include <stdlib.h>


int main(int argc, char *argv[]) {
    int xMax = atoi(argv[1]);
    int xMin = atoi(argv[2]);
    int yMax = atoi(argv[3]);
    int yMin = atoi(argv[4]);
    int zMax = atoi(argv[5]);
    int zMin = atoi(argv[6]);

    int xMid = (xMax + xMin)/2;
    int yMid = (yMax + yMin)/2;
    int zMid = (zMax + zMin)/2;

    printf("X offset: -%d\n", xMid);
    printf("Y offset: -%d\n", yMid);
    printf("Z offset: -%d\n", zMid);

    int xAdjMax = xMax - xMid;
    int yAdjMax = yMax - yMid;
    int zAdjMax = zMax - zMid;

    double xScale = 1;
    double yScale = (double) xAdjMax/yAdjMax;
    double zScale = (double) xAdjMax/zAdjMax;

    printf("X scale factor: %f\n", xScale);
    printf("Y scale factor: %f\n", yScale);
    printf("Z scale factor: %f\n", zScale);

    return 0;
}

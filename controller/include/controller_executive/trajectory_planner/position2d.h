#ifndef CONTROLLER_EXECUTIVE_POSITION2D_H
#define CONTROLLER_EXECUTIVE_POSITION2D_H

namespace trajectory_planner {

class Position2DInt {

 public:

  Position2DInt() {
    x = 0;
    y = 0;
  }

  Position2DInt(long _x, long _y) {
    x = _x;
    y = _y;
  }

  Position2DInt(const Position2DInt &p) {
    x = p.x;
    y = p.y;
  }

  long x;
  long y;

};

}

#endif //CONTROLLER_EXECUTIVE_POSITION2D_H

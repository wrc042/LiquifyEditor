#pragma once
#include "viewer/rlcore.hpp"

class RLEditor : public RLCore {
  public:
  protected:
    void init_callback(int bufnum){};
    void update_callback(int bufnum){};

  private:
};

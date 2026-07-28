#include "6502.h"

void setup() {
  cpu_reset();
}

void loop() {
  cpu_step();
}
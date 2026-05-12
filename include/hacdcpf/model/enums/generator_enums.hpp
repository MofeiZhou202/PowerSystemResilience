#pragma once

namespace hacdcpf {

// Generator cost model types
enum class GenCostModel {
  Polynomial = 2,        // f(Pg) = c2*Pg^2 + c1*Pg + c0
  PiecewiseLinear = 1,
};

// Generator fuel / technology types
enum class FuelType {
  Unknown = 0,
  Coal,
  Gas,
  Oil,
  Nuclear,
  Hydro,
  Wind,
  Solar,
  Biomass,
  Geothermal,
  Storage,
};

}  // namespace hacdcpf

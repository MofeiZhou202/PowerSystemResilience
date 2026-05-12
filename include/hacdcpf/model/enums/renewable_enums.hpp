#pragma once

namespace hacdcpf {

// Renewable generation type
enum class RenewableType {
  Wind = 0,
  SolarPV = 1,
  SolarCSP = 2,
  Hydro = 3,
};

// Static generator type (distributed generation)
enum class SgenType {
  PV = 0,       // solar PV inverter
  Wind = 1,     // wind turbine
  CHP = 2,      // combined heat & power
  Diesel = 3,   // diesel genset
  FuelCell = 4, // fuel cell
  Other = 5,
};

// PV system inverter control mode
enum class PVControlMode {
  MPPT = 0,         // max power point tracking
  PQ = 1,           // fixed P, Q
  VQ = 2,           // voltage-reactive
  Curtailed = 3,    // curtailed output
};

}  // namespace hacdcpf

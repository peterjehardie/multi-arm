// Physical constants and material data (SI units throughout: m, kg, s, A, V, K).

export const G0 = 9.80665;             // standard gravity [m/s^2]
export const T0 = 273.15;              // 0 degC in kelvin
export const AMBIENT = T0 + 22;        // workshop air temperature [K]
export const C_LIGHT = 299792458;      // [m/s]

export const COPPER = {
  rho20: 1.68e-8,        // resistivity at 20 degC [ohm m]
  alpha: 0.00393,        // temperature coefficient of resistance [1/K]
  density: 8960,         // [kg/m^3]
  cp: 385,               // specific heat [J/(kg K)]
};

export const ALUMINIUM = { density: 2700, cp: 897, k: 167 };
export const BRASS = { density: 8500, cp: 380, k: 110 };
export const STEEL = { density: 7850, cp: 490, k: 50 };

// American Wire Gauge: solid-equivalent diameter.
export function awgDiameter(awg) {
  return 0.127e-3 * Math.pow(92, (36 - awg) / 39);
}
export function awgArea(awg) {
  const d = awgDiameter(awg);
  return (Math.PI * d * d) / 4;
}

export const degC = (K) => K - T0;
export const K = (C) => C + T0;
export const rpm = (w) => (w * 60) / (2 * Math.PI);

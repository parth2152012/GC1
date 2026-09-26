/// Enforces the 20m minimum inter-vehicle safety separation constraint.
/// Coordinates are provided as local ENU tuples: (x_meters, y_meters).
pub fn check_safety_distance(drone_a: (f64, f64), drone_b: (f64, f64)) -> bool {
    let dist = enu_distance_meters(drone_a, drone_b);
    dist >= 20.0 // Returns true if distance is safe (>= 20m)
}

pub fn enu_distance_meters(a: (f64, f64), b: (f64, f64)) -> f64 {
    let dx = a.0 - b.0;
    let dy = a.1 - b.1;
    (dx * dx + dy * dy).sqrt()
}

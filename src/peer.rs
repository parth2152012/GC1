/// A local East/North/Up position in metres, relative to the swarm origin.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PositionEnu {
    pub east_m: f64,
    pub north_m: f64,
    pub up_m: f64,
}

pub const MIN_SEPARATION_M: f64 = 20.0;
pub const RANGE_WARNING_M: f64 = 85.0;

/// Computes the 3D Euclidean distance used for collision and link-range decisions.
pub fn enu_distance_meters(a: PositionEnu, b: PositionEnu) -> f64 {
    let east = a.east_m - b.east_m;
    let north = a.north_m - b.north_m;
    let up = a.up_m - b.up_m;
    (east.mul_add(east, north.mul_add(north, up * up))).sqrt()
}

pub fn is_separation_breach(a: PositionEnu, b: PositionEnu) -> bool {
    enu_distance_meters(a, b) < MIN_SEPARATION_M
}

pub fn needs_link_throttle(a: PositionEnu, b: PositionEnu) -> bool {
    enu_distance_meters(a, b) >= RANGE_WARNING_M
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn distance_is_three_dimensional_and_thresholds_are_exact() {
        let origin = PositionEnu {
            east_m: 0.0,
            north_m: 0.0,
            up_m: 0.0,
        };
        let point = PositionEnu {
            east_m: 12.0,
            north_m: 16.0,
            up_m: 15.0,
        };
        assert_eq!(enu_distance_meters(origin, point), 25.0);
        assert!(!is_separation_breach(
            origin,
            PositionEnu {
                east_m: 20.0,
                ..origin
            }
        ));
        assert!(is_separation_breach(
            origin,
            PositionEnu {
                east_m: 19.999,
                ..origin
            }
        ));
        assert!(needs_link_throttle(
            origin,
            PositionEnu {
                east_m: 85.0,
                ..origin
            }
        ));
    }
}

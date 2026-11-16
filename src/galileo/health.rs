#[cfg(doc)]
use crate::galileo::{GalileoInavWord3, GalileoInavWord5};

/// Galileo Signal Health Status (OS SIS ICD v2.1, table 82),
/// transmitted on 2 bits for each signal (E1-B, E5a, E5b) in [GalileoInavWord5].
#[derive(Debug, Default, Copy, Clone, PartialEq, Eq)]
pub enum GalileoSignalHealth {
    /// Signal OK
    #[default]
    Ok,

    /// Signal out of service
    OutOfService,

    /// Signal will be out of service, or is transmitted
    /// in Extended Operations Mode (EOM).
    ExtendedOperations,

    /// Signal component currently in test
    InTest,
}

impl GalileoSignalHealth {
    /// Builds a [GalileoSignalHealth] from its 2-bit binary representation.
    pub fn decode(value: u8) -> Self {
        match value & 0x3 {
            0 => Self::Ok,
            1 => Self::OutOfService,
            2 => Self::ExtendedOperations,
            _ => Self::InTest,
        }
    }

    /// Returns the 2-bit binary representation of this [GalileoSignalHealth].
    pub fn encode(&self) -> u8 {
        match self {
            Self::Ok => 0,
            Self::OutOfService => 1,
            Self::ExtendedOperations => 2,
            Self::InTest => 3,
        }
    }

    /// True if this signal may be used.
    pub fn is_ok(&self) -> bool {
        *self == Self::Ok
    }
}

/// Galileo Signal In Space Accuracy (SISA) index (OS SIS ICD v2.1, table 89),
/// transmitted on 8 bits in [GalileoInavWord3].
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct GalileoSisa(pub u8);

impl Default for GalileoSisa {
    /// Builds a [GalileoSisa] marking No Accuracy Prediction Available (NAPA).
    fn default() -> Self {
        Self::napa()
    }
}

impl GalileoSisa {
    /// Index marking No Accuracy Prediction Available (NAPA).
    pub const NAPA: u8 = 255;

    /// Builds a [GalileoSisa] marking No Accuracy Prediction Available (NAPA).
    pub fn napa() -> Self {
        Self(Self::NAPA)
    }

    /// True when No Accuracy Prediction is Available: the satellite should not be used.
    pub fn is_napa(&self) -> bool {
        self.0 == Self::NAPA
    }

    /// Returns the accuracy (in meters) this index stands for,
    /// or None for spare indexes and NAPA.
    pub fn meters(&self) -> Option<f64> {
        let index = self.0 as f64;

        match self.0 {
            0..=49 => Some(index * 0.01),
            50..=74 => Some(0.5 + (index - 50.0) * 0.02),
            75..=99 => Some(1.0 + (index - 75.0) * 0.04),
            100..=125 => Some(2.0 + (index - 100.0) * 0.16),
            _ => None,
        }
    }

    /// Builds the smallest [GalileoSisa] index whose accuracy is equal to or worse than
    /// `meters` (in meters). Values above 6 m (or invalid values) give NAPA.
    pub fn from_meters(meters: f64) -> Self {
        if !(0.0..=6.0).contains(&meters) {
            return Self::napa();
        }

        // tolerance on the floating point boundaries
        let meters = meters - 1.0e-9;

        let index = if meters < 0.5 {
            (meters / 0.01).ceil()
        } else if meters < 1.0 {
            50.0 + ((meters - 0.5) / 0.02).ceil()
        } else if meters < 2.0 {
            75.0 + ((meters - 1.0) / 0.04).ceil()
        } else {
            100.0 + ((meters - 2.0) / 0.16).ceil()
        };

        Self(index.max(0.0) as u8)
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn signal_health() {
        for value in 0..4 {
            assert_eq!(GalileoSignalHealth::decode(value).encode(), value);
        }
        assert!(GalileoSignalHealth::decode(0).is_ok());
        assert_eq!(
            GalileoSignalHealth::decode(2),
            GalileoSignalHealth::ExtendedOperations
        );
    }

    #[test]
    fn sisa() {
        assert!(GalileoSisa::default().is_napa());
        assert_eq!(GalileoSisa::default().meters(), None);
        assert_eq!(GalileoSisa(126).meters(), None);

        for (index, meters) in [
            (0, 0.0),
            (49, 0.49),
            (50, 0.5),
            (74, 0.98),
            (75, 1.0),
            (99, 1.96),
            (100, 2.0),
            (107, 3.12),
            (125, 6.0),
        ] {
            let sisa = GalileoSisa(index);
            assert!((sisa.meters().unwrap() - meters).abs() < 1e-9, "{}", index);
            assert_eq!(GalileoSisa::from_meters(meters), sisa, "{} m", meters);
        }

        assert!(GalileoSisa::from_meters(6.1).is_napa());
        assert!(GalileoSisa::from_meters(-1.0).is_napa());
        assert_eq!(GalileoSisa::from_meters(3.0), GalileoSisa(107));
    }
}

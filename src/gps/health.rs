/// Health mask for a single satellite
#[derive(Debug, Default, Copy, Clone, PartialEq, BitWrite, BitRead)]
pub struct GpsQzssSatelliteHealth {
    /// 6-bit mask
    #[size = 6]
    inner: u8,   
}

impl From<u8> for GpsQzssSatelliteHealth {
    fn from(value: u8) -> Self {
        Self {
            inner: value & 0x3f,
        }
    }
}

impl GpsQzssSateliteHealth {
    /// True if this mask marks a healthy satellite.
    pub fn is_healthy(&self) -> bool {
        self.inner == 0
    }
    
    pub fn is_unavailable(&self) -> bool {
        self.inner == 0x1C
    }

    pub fn under_maintenance(&self) -> bool {
        self.inner == 0x1D
    }
    
    pub fn has_transmission_issues(&self) -> bool {
        !self.is_healthy()
        && !self.pending_maintenance()
        && !self.is_unavailable()
        && self.inner != 0x1E
        && self.inner != 0x1F
    }

    /// Returns a healthy [GpsQzssSatelliteHealth] mask.
    pub fn healthy() -> Self {
        Self::from(0u8)
    }

    /// Returns a [GpsQzssSatelliteHealth] mask marking satellite 
    /// unavailability.
    pub fn unavailable() -> Self {
        Self::from(0x1C)
    }

    /// Returns a [GpsQzssSatelliteHealth] mask marking pending satellite 
    /// maintenance operations.
    pub fn maintenance() -> Self {
        Self::from(0x1D)
    }
    
    /// Returns a [GpsQzssSatelliteHealth] mask marking satellite transmission issues.
    pub fn transmission_issues() -> Self {
        Self::from(0x0C)
    }
}

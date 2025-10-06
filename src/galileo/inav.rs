/// [GalIRaw] is the interal representation
pub(crate) struct GalIRaw {}

/// [GalE5bINAV] is the I/NAV message interpretation
/// of the E5b-I signal.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct GalE5bINAV {
    /// 64-bit reserved #1
    // TODO: not protected by CRC
    pub reserved1: u64,

    /// 8-bit reserved #2
    pub reserved2: u8,

    /// 16-bit data payload
    pub data: [u8; 16],
    // TODO tail bits (6bits)
    // defined in 4.3.2.2
    // these bits are not protected by the CRC
}

/// [GalE1bINAV] is the I/NAV message interpretation
/// of the E1-B signal.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct GalE1bINAV {
    /// 16-bit data payload
    pub data: [u8; 16],

    // TODO not protected by CRC
    pub SSP: u8,

    /// 40-bit OSNMA
    pub osnma: [u8; 5],
    // TODO tail bits (6bits)
    // defined in 4.3.2.2
    // these bits are not protected by the CRC
}

/// [GalINavMessage] is the message wrapper that may
/// have different forms depending on the received signal.
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum GalINAV {
    /// [GalE1bINAV] is the I/NAV message interpretation
    /// of the E1-B signal.
    E1b(GalE1bINAV),

    /// [GalE5bINAV] is the I/NAV message interpretation
    /// of the E5b signal.
    E5b(GalE5bINAV),
}

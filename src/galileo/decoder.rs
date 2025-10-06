use crate::galileo::GalileoMessage;

/// Known Galileo signals, because full message interpretation
/// depends on the received signal.
#[derive(Debug, Copy, Clone, PartialEq, Default)]
enum Signal {
    #[default]
    E1,
    E5,
}

/// [SyncByte]s we can search for, without any preference.
#[derive(Debug, Copy, Clone, PartialEq, Default)]
enum SyncByte {
    #[default]
    INAV,

    FNAV,
}

impl SyncByte {
    /// Returns the binary value of this [SyncByte]
    pub fn value(&self) -> u16 {
        match self {
            Self::INAV => 0x0260,
            Self::FNAV => 0x0B70,
        }
    }
}

#[derive(Debug, Default, Copy, Clone, PartialEq)]
enum State {
    /// Searching for a sync byte whatever its genre
    #[default]
    SyncByte,

    /// Collecting message and signal dependent bits
    Collecting(usize),
}

#[derive(Copy, Clone, Default)]
pub struct GalileoDecoder {
    /// [SyncByte] that was found
    sync_byte: SyncByte,

    /// Current [State]
    state: State,

    /// [Signal] being received
    signal: Signal,
}

impl GalileoDecoder {
    /// Creates a new E1 signal decoder.
    /// ```
    /// // Create a new E5 signal decoder
    /// let mut decoder = GalileoDecoder::default()
    ///     .e1();
    /// ```
    pub fn e1(mut self) -> Self {
        self.signal = Signal::E1;
        self.state = Default::default();
        self
    }

    /// Creates a new E5 signal decoder
    /// ```
    /// // Create a new E5 signal decoder
    /// let mut decoder = GalileoDecoder::default()
    ///     .e5();
    /// ```
    pub fn e5(mut self) -> Self {
        self.signal = Signal::E5;
        self.state = Default::default();
        self
    }

    /// Attempts at decoding a new [GalMessage].
    /// Returns the total number of consumed bytes (at all times).
    /// Returns possible decoded [GalileoMessage].
    pub fn decode(&mut self, buffer: &[u8]) -> (usize, Option<GalileoMessage>) {
        let next_state = match self.state {
            State::SyncByte => {
                // we need to gather 10 or 12 bits
                // and identify either of both sync bytes

                self.state // TODO
            },
            State::Collecting(collected) => {
                // we need to collect all bits and run
                // the interpretation algorithm
                self.state // TODO
            },
        };

        self.state = next_state;

        (0, None) // TODO
    }
}

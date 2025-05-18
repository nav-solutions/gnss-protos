#[cfg(feature = "std")]
use std::sync::Once;

#[cfg(feature = "std")]
use log::LevelFilter;

#[cfg(feature = "std")]
static INIT: Once = Once::new();

pub fn init_logger() {
    #[cfg(feature = "std")]
    INIT.call_once(|| {
        env_logger::builder()
            .is_test(true)
            .filter_level(LevelFilter::Trace)
            .init();
    });
}

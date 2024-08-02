use std::time::Duration;

/// A sequence of strongly monotonic inrceasing durations. Introduced for testing conversions from
/// `Duration` to platform-specific types.
pub(crate) const MONOTONIC_DURATIONS: [Duration; 17] = [
    Duration::ZERO,
    Duration::from_nanos(1),
    Duration::from_millis(1),
    Duration::from_secs(1),
    Duration::from_secs(i16::MAX as u64 - 1),
    Duration::from_secs(i16::MAX as u64),
    Duration::from_secs(i16::MAX as u64 + 1),
    Duration::from_secs(i32::MAX as u64 - 1),
    Duration::from_secs(i32::MAX as u64),
    Duration::from_secs(i32::MAX as u64 + 1),
    Duration::from_secs(i64::MAX as u64 - 1),
    Duration::from_secs(i64::MAX as u64),
    Duration::from_secs(i64::MAX as u64 + 1),
    Duration::from_secs(u64::MAX - 1),
    Duration::from_secs(u64::MAX),
    Duration::new(u64::MAX, 1_000_000),
    Duration::MAX,
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn basic_durations_properties() {
        assert_eq!(Duration::ZERO, *MONOTONIC_DURATIONS.first().unwrap());
        assert_eq!(Duration::MAX, *MONOTONIC_DURATIONS.last().unwrap());

        // Check that this array is monotonic.
        let mut last = MONOTONIC_DURATIONS[0];
        for next in MONOTONIC_DURATIONS {
            assert!(last <= next);
            last = next;
        }
    }
}

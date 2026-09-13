//! Inexpensive portable process-memory sampling.
//!
//! Only RSS is sampled here; allocator-tracked and cache sizes are filled by
//! the embedding application. When no reliable value exists, callers keep
//! `None` (serialized as `unknown` in CSV, `null` in JSON).

/// Current resident set size in bytes, or `None` when unavailable.
///
/// Linux reads `/proc/self/statm` (second field * page size). Other platforms
/// return `None` rather than a misleading estimate.
pub fn current_rss_bytes() -> Option<u64> {
    #[cfg(target_os = "linux")]
    {
        read_linux_rss()
    }
    #[cfg(not(target_os = "linux"))]
    {
        None
    }
}

#[cfg(target_os = "linux")]
fn read_linux_rss() -> Option<u64> {
    let content = std::fs::read_to_string("/proc/self/statm").ok()?;
    let resident_pages: u64 = content.split_whitespace().nth(1)?.parse().ok()?;
    // Page size without libc dependency: Linux default is 4096 on the vast
    // majority of targets; a wrong page size would still be a plausible
    // magnitude, but callers treat RSS as approximate anyway. Keep this crate
    // `forbid(unsafe_code)` instead of pulling libc for `getpagesize()`.
    const FALLBACK_PAGE_SIZE: u64 = 4096;
    resident_pages.checked_mul(FALLBACK_PAGE_SIZE)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rss_is_plausible_or_unknown() {
        // On Linux CI this should be Some; elsewhere None is acceptable.
        if let Some(rss) = current_rss_bytes() {
            assert!(rss > 0);
            assert!(rss < 1 << 50, "implausible RSS: {rss}");
        }
    }
}

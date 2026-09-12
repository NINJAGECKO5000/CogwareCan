// SPDX-License-Identifier: GPL-3.0-only
//! IEEE 802.3 CRC-32 (reflected, polynomial 0xEDB88320), bitwise so it costs
//! no table space on the microcontroller. Used by `xfer` to verify images.

pub struct Crc32(u32);

impl Crc32 {
    pub const fn new() -> Self {
        Crc32(0xFFFFFFFF)
    }

    pub fn update(&mut self, data: &[u8]) {
        for &byte in data {
            let mut crc = self.0 ^ (byte as u32);
            for _ in 0..8 {
                if crc & 1 != 0 {
                    crc = (crc >> 1) ^ 0xEDB88320;
                } else {
                    crc >>= 1;
                }
            }
            self.0 = crc;
        }
    }

    pub fn finish(&self) -> u32 {
        self.0 ^ 0xFFFFFFFF
    }
}

impl Default for Crc32 {
    fn default() -> Self {
        Self::new()
    }
}

pub fn crc32(data: &[u8]) -> u32 {
    let mut c = Crc32::new();
    c.update(data);
    c.finish()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc32_check_value() {
        assert_eq!(crc32(b"123456789"), 0xCBF43926);
    }

    #[test]
    fn test_incremental_matches_one_shot() {
        let data = b"123456789";
        let mut c = Crc32::new();
        c.update(&data[..5]);
        c.update(&data[5..]);
        assert_eq!(c.finish(), 0xCBF43926);
    }
}

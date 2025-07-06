//! GPS Module for u-blox NEO M9N
//! 
//! This module provides basic UBX packet parsing for GPS data

use defmt::*;

/// Simple GPS data structure
#[derive(Debug, Clone)]
pub struct GpsData {
    pub latitude: f64,   // degrees
    pub longitude: f64,  // degrees
    pub altitude: i32,   // millimeters above sea level
    pub satellites: u8,  // number of satellites used
    pub fix_valid: bool, // whether we have a valid GPS fix
    // GPS time components
    pub year: u16,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
}

impl Default for GpsData {
    fn default() -> Self {
        Self {
            latitude: 0.0,
            longitude: 0.0,
            altitude: 0,
            satellites: 0,
            fix_valid: false,
            year: 1970,
            month: 1,
            day: 1,
            hour: 0,
            minute: 0,
            second: 0,
        }
    }
}

impl GpsData {
    // Format display method removed - using direct formatting in main.rs
}

/// UBX packet parser
pub struct UbxParser {
    buffer: [u8; 256],
    buffer_pos: usize,
    state: ParserState,
}

#[derive(Clone, Copy)]
enum ParserState {
    WaitingForSync1,
    WaitingForSync2,
    ReadingHeader,
    ReadingPayload,
}

impl UbxParser {
    pub fn new() -> Self {
        Self {
            buffer: [0; 256],
            buffer_pos: 0,
            state: ParserState::WaitingForSync1,
        }
    }

    /// Process incoming UART byte and return GPS data if a complete packet is parsed
    pub fn process_byte(&mut self, byte: u8) -> Option<GpsData> {
        match self.state {
            ParserState::WaitingForSync1 => {
                if byte == 0xB5 {  // UBX sync char 1
                    self.buffer[0] = byte;
                    self.buffer_pos = 1;
                    self.state = ParserState::WaitingForSync2;
                }
            }
            ParserState::WaitingForSync2 => {
                if byte == 0x62 {  // UBX sync char 2
                    self.buffer[1] = byte;
                    self.buffer_pos = 2;
                    self.state = ParserState::ReadingHeader;
                } else {
                    self.state = ParserState::WaitingForSync1;
                }
            }
            ParserState::ReadingHeader => {
                self.buffer[self.buffer_pos] = byte;
                self.buffer_pos += 1;
                
                if self.buffer_pos >= 6 {  // We have class, id, and length
                    let payload_length = u16::from_le_bytes([self.buffer[4], self.buffer[5]]) as usize;
                    
                    if payload_length > 200 {  // Sanity check - too big
                        self.state = ParserState::WaitingForSync1;
                        return None;
                    }
                    
                    if payload_length == 0 {
                        // No payload, check if this is a message we care about
                        self.state = ParserState::WaitingForSync1;
                        return None;
                    }
                    
                    self.state = ParserState::ReadingPayload;
                }
            }
            ParserState::ReadingPayload => {
                self.buffer[self.buffer_pos] = byte;
                self.buffer_pos += 1;
                
                let payload_length = u16::from_le_bytes([self.buffer[4], self.buffer[5]]) as usize;
                let expected_total_length = 6 + payload_length + 2; // header + payload + checksum
                
                if self.buffer_pos >= expected_total_length {
                    // We have a complete packet, try to parse it
                    let result = self.parse_packet();
                    self.state = ParserState::WaitingForSync1;
                    return result;
                }
            }
        }
        None
    }

    fn parse_packet(&self) -> Option<GpsData> {
        if self.buffer_pos < 8 {
            return None;
        }

        let class = self.buffer[2];
        let id = self.buffer[3];
        let payload_length = u16::from_le_bytes([self.buffer[4], self.buffer[5]]) as usize;

        // Check if we have enough data
        if self.buffer_pos < 6 + payload_length + 2 {
            return None;
        }

        // Verify checksum
        if !self.verify_checksum(payload_length) {
            warn!("UBX checksum failed");
            return None;
        }

        // Parse NAV-PVT message (0x01, 0x07)
        if class == 0x01 && id == 0x07 && payload_length >= 84 {
            return self.parse_nav_pvt();
        }

        None
    }

    fn verify_checksum(&self, payload_length: usize) -> bool {
        let mut ck_a = 0u8;
        let mut ck_b = 0u8;

        // Calculate checksum over class, id, length, and payload
        for i in 2..(6 + payload_length) {
            ck_a = ck_a.wrapping_add(self.buffer[i]);
            ck_b = ck_b.wrapping_add(ck_a);
        }

        let expected_ck_a = self.buffer[6 + payload_length];
        let expected_ck_b = self.buffer[6 + payload_length + 1];

        ck_a == expected_ck_a && ck_b == expected_ck_b
    }

    fn parse_nav_pvt(&self) -> Option<GpsData> {
        let payload_start = 6;
        
        // Extract fix type (offset 20 in payload)
        let fix_type = self.buffer[payload_start + 20];
        let fix_valid = fix_type >= 2; // 2D or 3D fix
        
        // Extract number of satellites (offset 23 in payload)
        let satellites = self.buffer[payload_start + 23];
        
        // Extract longitude (offset 24, 4 bytes, little-endian, scaled by 1e-7)
        let lon_raw = i32::from_le_bytes([
            self.buffer[payload_start + 24],
            self.buffer[payload_start + 25],
            self.buffer[payload_start + 26],
            self.buffer[payload_start + 27],
        ]);
        let longitude = (lon_raw as f64) * 1e-7;
        
        // Extract latitude (offset 28, 4 bytes, little-endian, scaled by 1e-7)
        let lat_raw = i32::from_le_bytes([
            self.buffer[payload_start + 28],
            self.buffer[payload_start + 29],
            self.buffer[payload_start + 30],
            self.buffer[payload_start + 31],
        ]);
        let latitude = (lat_raw as f64) * 1e-7;
        
        // Extract height above sea level (offset 36, 4 bytes, little-endian, in mm)
        let altitude = i32::from_le_bytes([
            self.buffer[payload_start + 36],
            self.buffer[payload_start + 37],
            self.buffer[payload_start + 38],
            self.buffer[payload_start + 39],
        ]);

        // Extract time of fix (UTC) (offset 40, 8 bytes, little-endian)
        let year = u16::from_le_bytes([
            self.buffer[payload_start + 40],
            self.buffer[payload_start + 41],
        ]);
        let month = self.buffer[payload_start + 42];
        let day = self.buffer[payload_start + 43];
        let hour = self.buffer[payload_start + 44];
        let minute = self.buffer[payload_start + 45];
        let second = self.buffer[payload_start + 46];

        Some(GpsData {
            latitude,
            longitude,
            altitude,
            satellites,
            fix_valid,
            year,
            month,
            day,
            hour,
            minute,
            second,
        })
    }
}

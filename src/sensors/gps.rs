//! Simple GPS Module 
//! Outputs one CSV message: GPS: MM/DD/YYYY, HH:MM:SS, LAT, LONG, ALT, SATS, FIX

use rp235x_hal as hal;
use hal::uart::{DataBits, StopBits, UartConfig, UartPeripheral, Enabled};
use hal::gpio::{FunctionUart, Pin, bank0::{Gpio0, Gpio1}, PullDown};
use hal::pac;
use hal::Clock;
use embedded_hal_nb::serial::Read;
use defmt::*;
use core::fmt::Write;

// UBX sync characters
const UBX_SYNC_CHAR_1: u8 = 0xB5;
const UBX_SYNC_CHAR_2: u8 = 0x62;

// Simple GPS data structure
#[derive(Default)]
pub struct GpsData {
    pub year: u16,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
    pub latitude: i32,    // 1e-7 degrees
    pub longitude: i32,   // 1e-7 degrees
    pub altitude: i32,    // mm
    pub satellites: u8,
    pub fix_type: u8,
}

impl GpsData {
    pub fn format_csv(&self) -> heapless::String<128> {
        let mut output = heapless::String::new();
        // Convert coordinates from 1e-7 degrees to more readable format
        // Split into whole degrees and fractional parts
        let lat_whole = self.latitude / 10_000_000;
        let lat_frac = (self.latitude % 10_000_000).abs();
        let lon_whole = self.longitude / 10_000_000;
        let lon_frac = (self.longitude % 10_000_000).abs();
        let alt_m = self.altitude / 1000; // Convert mm to meters
        
        let _ = core::write!(output, "{:02}/{:02}/{:04}, {:02}:{:02}:{:02}, {}.{:07}°, {}.{:07}°, {}m, {} sats, fix:{}",
            self.month, self.day, self.year,
            self.hour, self.minute, self.second,
            lat_whole, lat_frac, lon_whole, lon_frac, alt_m,
            self.satellites, self.fix_type);
        output
    }
}

// Simple GPS module
pub struct GpsModule {
    uart: Option<UartPeripheral<Enabled, pac::UART0, (Pin<Gpio0, FunctionUart, PullDown>, Pin<Gpio1, FunctionUart, PullDown>)>>,
    parser: UbxParser,
    last_gps_data: GpsData,
}

impl Default for GpsModule {
    fn default() -> Self {
        Self::new()
    }
}

impl GpsModule {
    pub fn new() -> Self {
        Self { 
            uart: None,
            parser: UbxParser::new(),
            last_gps_data: GpsData::default(),
        }
    }
    
    pub fn init(
        &mut self,
        uart0: pac::UART0,
        gpio0: Pin<hal::gpio::bank0::Gpio0, hal::gpio::FunctionNull, hal::gpio::PullDown>,
        gpio1: Pin<hal::gpio::bank0::Gpio1, hal::gpio::FunctionNull, hal::gpio::PullDown>,
        resets: &mut pac::RESETS,
        clocks: &hal::clocks::ClocksManager,
    ) -> Result<(), hal::uart::Error> {
        let tx_pin = gpio0.into_function::<FunctionUart>();
        let rx_pin = gpio1.into_function::<FunctionUart>();
        
        let uart_config = UartConfig::new(
            hal::fugit::HertzU32::from_raw(38_400),
            DataBits::Eight,
            None,
            StopBits::One,
        );
        
        let uart = UartPeripheral::new(uart0, (tx_pin, rx_pin), resets)
            .enable(uart_config, clocks.peripheral_clock.freq())?;
        
        self.uart = Some(uart);
        Ok(())
    }
    
    pub fn update(&mut self) {
        if let Some(ref mut uart) = self.uart {
            // Read bytes and parse
            for _ in 0..50 { // Limit processing per update
                match uart.read() {
                    Ok(byte) => {
                        if let Some(gps_data) = self.parser.parse_byte(byte) {
                            self.last_gps_data = gps_data;
                            // Output the simple CSV message
                            info!("GPS: {}", self.last_gps_data.format_csv().as_str());
                        }
                    }
                    Err(_) => break,
                }
            }
        }
    }
    
    pub fn get_last_data(&self) -> &GpsData {
        &self.last_gps_data
    }
}

// Simple UBX parser states
#[derive(PartialEq)]
enum ParserState {
    Sync1,
    Sync2,
    Header,
    Payload,
    Checksum,
}

pub struct UbxParser {
    state: ParserState,
    header: [u8; 4],  // class, id, len_low, len_high
    payload: [u8; 92], // Only need NAV-PVT payload
    header_idx: usize,
    payload_idx: usize,
    payload_len: u16,
    checksum_a: u8,
    checksum_b: u8,
    calc_checksum_a: u8,
    calc_checksum_b: u8,
    checksum_idx: u8,  // Counter for checksum bytes
}

impl UbxParser {
    pub fn new() -> Self {
        Self {
            state: ParserState::Sync1,
            header: [0; 4],
            payload: [0; 92],
            header_idx: 0,
            payload_idx: 0,
            payload_len: 0,
            checksum_a: 0,
            checksum_b: 0,
            calc_checksum_a: 0,
            calc_checksum_b: 0,
            checksum_idx: 0,
        }
    }
    
    fn reset(&mut self) {
        self.state = ParserState::Sync1;
        self.header_idx = 0;
        self.payload_idx = 0;
        self.payload_len = 0;
        self.calc_checksum_a = 0;
        self.calc_checksum_b = 0;
        self.checksum_idx = 0;
    }
    
    fn add_checksum(&mut self, byte: u8) {
        self.calc_checksum_a = self.calc_checksum_a.wrapping_add(byte);
        self.calc_checksum_b = self.calc_checksum_b.wrapping_add(self.calc_checksum_a);
    }
    
    pub fn parse_byte(&mut self, byte: u8) -> Option<GpsData> {
        match self.state {
            ParserState::Sync1 => {
                if byte == UBX_SYNC_CHAR_1 {
                    self.state = ParserState::Sync2;
                }
            }
            ParserState::Sync2 => {
                if byte == UBX_SYNC_CHAR_2 {
                    self.state = ParserState::Header;
                    self.header_idx = 0;
                    self.calc_checksum_a = 0;
                    self.calc_checksum_b = 0;
                } else {
                    self.reset();
                    if byte == UBX_SYNC_CHAR_1 {
                        self.state = ParserState::Sync2;
                    }
                }
            }
            ParserState::Header => {
                self.header[self.header_idx] = byte;
                self.add_checksum(byte);
                self.header_idx += 1;
                
                if self.header_idx == 4 {
                    self.payload_len = (self.header[3] as u16) << 8 | (self.header[2] as u16);
                    
                    // Only parse NAV-PVT messages (class 0x01, id 0x07, length 92)
                    if self.header[0] == 0x01 && self.header[1] == 0x07 && self.payload_len == 92 {
                        self.state = ParserState::Payload;
                        self.payload_idx = 0;
                    } else {
                        self.reset();
                    }
                }
            }
            ParserState::Payload => {
                self.payload[self.payload_idx] = byte;
                self.add_checksum(byte);
                self.payload_idx += 1;
                
                if self.payload_idx as u16 == self.payload_len {
                    self.state = ParserState::Checksum;
                    self.checksum_idx = 0;  // Reset checksum counter
                }
            }
            ParserState::Checksum => {
                if self.checksum_idx == 0 {
                    self.checksum_a = byte;
                    self.checksum_idx = 1;
                } else {
                    self.checksum_b = byte;
                    
                    if self.calc_checksum_a == self.checksum_a && self.calc_checksum_b == self.checksum_b {
                        let result = self.parse_nav_pvt();
                        self.reset();
                        return result;
                    } else {
                        self.reset();
                    }
                }
            }
        }
        None
    }
    
    fn parse_nav_pvt(&self) -> Option<GpsData> {
        let p = &self.payload;
        let mut gps = GpsData::default();
        
        // Extract basic time/position data from UBX-NAV-PVT
        gps.year = u16::from_le_bytes([p[4], p[5]]);
        gps.month = p[6];
        gps.day = p[7];
        gps.hour = p[8];
        gps.minute = p[9];
        gps.second = p[10];
        gps.fix_type = p[20];
        gps.satellites = p[23];
        gps.longitude = i32::from_le_bytes([p[24], p[25], p[26], p[27]]);
        gps.latitude = i32::from_le_bytes([p[28], p[29], p[30], p[31]]);
        gps.altitude = i32::from_le_bytes([p[36], p[37], p[38], p[39]]);
        
        Some(gps)
    }
}

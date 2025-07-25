//! Simple GPS Module 
//! Outputs one CSV message: GPS: MM/DD/YYYY, HH:MM:SS, LAT, LONG, ALT, SATS, FIX

use rp235x_hal as hal;
use hal::uart::{UartPeripheral, Enabled};
use hal::pac::UART0;
use hal::gpio::{Pin, bank0::{Gpio0, Gpio1}, PullDown, FunctionUart};
use heapless::String as HeaplessString;
use embedded_hal_nb::serial::Read;
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

pub struct GpsModule {
    parser: UbxParser,
    last_gps_data: GpsData,
}

impl GpsModule {
    pub fn new() -> Self {
        Self {
            parser: UbxParser::new(),
            last_gps_data: GpsData::default(),
        }
    }

    pub fn update(&mut self, uart: &mut UartPeripheral<Enabled, UART0, (Pin<Gpio0, FunctionUart, PullDown>, Pin<Gpio1, FunctionUart, PullDown>)>) {
        for _ in 0..50 {
            if let Ok(byte) = uart.read() {
                if let Some(data) = self.parser.parse_byte(byte) {
                    self.last_gps_data = data;
                }
            } else {
                break;
            }
        }
    }

    pub fn get_last_data(&self) -> &GpsData {
        &self.last_gps_data
    }
}

// SensorDriver trait stub for demonstration (implement as needed)
use core::ops::DerefMut;
pub trait SensorDriver {
    type Bus;
    type RawData;
    type ParsedData;
    type Error;

    fn read_raw(&mut self, bus: impl DerefMut<Target = Self::Bus>) -> Result<Self::RawData, Self::Error>;
    fn parse(&self, raw: Self::RawData) -> Result<Self::ParsedData, Self::Error>;
}

impl SensorDriver for GpsModule {
    type Bus = UartPeripheral<Enabled, UART0, (Pin<Gpio0, FunctionUart, PullDown>, Pin<Gpio1, FunctionUart, PullDown>)>;
    type RawData = ();
    type ParsedData = GpsData;
    type Error = hal::uart::Error;

    fn read_raw(&mut self, mut bus: impl DerefMut<Target = Self::Bus>) -> Result<Self::RawData, Self::Error> {
        self.update(bus.deref_mut());
        Ok(())
    }

    fn parse(&self, _raw: Self::RawData) -> Result<Self::ParsedData, Self::Error> {
        Ok(self.last_gps_data)
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
// ...existing UBX parser code remains unchanged...
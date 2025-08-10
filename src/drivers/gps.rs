//! Minimal GPS driver: UART setup + NAV-PVT UBX parser yielding raw GpsData.

use heapless::String;
use rp235x_hal as hal;
use hal::uart::{DataBits, StopBits, UartConfig, UartPeripheral, Enabled};
use hal::gpio::{FunctionUart, Pin, bank0::{Gpio0, Gpio1}, PullDown};
use hal::pac;
use hal::Clock;
use embedded_hal_nb::serial::Read;
use core::fmt::Write as _; // for heapless String formatting

pub struct GpsDriver {
    uart: Option<UartPeripheral<Enabled, pac::UART0, (Pin<Gpio0, FunctionUart, PullDown>, Pin<Gpio1, FunctionUart, PullDown>)>>,
    parser: UbxParser,
    last: GpsData,
}

#[derive(Copy, Clone, Default)]
pub struct GpsData {
    pub year: u16,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
    pub latitude: i32,
    pub longitude: i32,
    pub altitude: i32,
    pub satellites: u8,
    pub fix_type: u8,
}

impl GpsData {
    pub fn format_csv(&self) -> String<128> {
        let mut out = String::new();
        let lat_whole = self.latitude / 10_000_000; let lat_frac = (self.latitude % 10_000_000).abs();
        let lon_whole = self.longitude / 10_000_000; let lon_frac = (self.longitude % 10_000_000).abs();
        let alt_m = self.altitude / 1000;
        let _ = core::write!(out, "{:02}/{:02}/{:04}, {:02}:{:02}:{:02}, {}.{:07}°, {}.{:07}°, {}m, {} sats, fix:{}",
            self.month,self.day,self.year,self.hour,self.minute,self.second,
            lat_whole,lat_frac,lon_whole,lon_frac,alt_m,self.satellites,self.fix_type);
        out
    }
}

impl GpsDriver {
    pub fn new() -> Self { Self { uart: None, parser: UbxParser::new(), last: GpsData::default() } }
    pub fn init(&mut self,
        uart0: pac::UART0,
        gpio0: Pin<hal::gpio::bank0::Gpio0, hal::gpio::FunctionNull, hal::gpio::PullDown>,
        gpio1: Pin<hal::gpio::bank0::Gpio1, hal::gpio::FunctionNull, hal::gpio::PullDown>,
        resets: &mut pac::RESETS,
        clocks: &hal::clocks::ClocksManager) -> Result<(), hal::uart::Error> {
        let tx_pin = gpio0.into_function::<FunctionUart>();
        let rx_pin = gpio1.into_function::<FunctionUart>();
        let uart_config = UartConfig::new(hal::fugit::HertzU32::from_raw(38_400), DataBits::Eight, None, StopBits::One);
        let uart = UartPeripheral::new(uart0, (tx_pin, rx_pin), resets)
            .enable(uart_config, clocks.peripheral_clock.freq())?;
        self.uart = Some(uart);
        Ok(())
    }
    pub fn poll(&mut self) {
        if let Some(ref mut uart) = self.uart {
            for _ in 0..50 {
                match uart.read() {
                    Ok(b) => { if let Some(d) = self.parser.parse_byte(b) { self.last = d; } }
                    Err(_) => break,
                }
            }
        }
    }
    pub fn last(&self) -> &GpsData { &self.last }
}

// Inline UBX parser (minimal subset)
#[derive(PartialEq)]
enum ParserState { Sync1, Sync2, Header, Payload, Checksum }

pub struct UbxParser {
    state: ParserState,
    header: [u8;4],
    payload: [u8;92],
    header_idx: usize,
    payload_idx: usize,
    payload_len: u16,
    checksum_a: u8,
    checksum_b: u8,
    calc_a: u8,
    calc_b: u8,
    cidx: u8,
}

const UBX_SYNC_CHAR_1: u8 = 0xB5; const UBX_SYNC_CHAR_2: u8 = 0x62;

impl UbxParser { pub fn new() -> Self { Self { state: ParserState::Sync1, header:[0;4], payload:[0;92], header_idx:0, payload_idx:0, payload_len:0, checksum_a:0, checksum_b:0, calc_a:0, calc_b:0, cidx:0 } }
    fn reset(&mut self){ self.state=ParserState::Sync1; self.header_idx=0; self.payload_idx=0; self.payload_len=0; self.calc_a=0; self.calc_b=0; self.cidx=0; }
    fn add_ck(&mut self,b:u8){ self.calc_a=self.calc_a.wrapping_add(b); self.calc_b=self.calc_b.wrapping_add(self.calc_a);}    
    pub fn parse_byte(&mut self, b:u8)->Option<GpsData>{ match self.state { ParserState::Sync1=>{ if b==UBX_SYNC_CHAR_1 { self.state=ParserState::Sync2; } }, ParserState::Sync2=>{ if b==UBX_SYNC_CHAR_2 { self.state=ParserState::Header; self.header_idx=0; self.calc_a=0; self.calc_b=0;} else { self.reset(); if b==UBX_SYNC_CHAR_1 { self.state=ParserState::Sync2; } } }, ParserState::Header=>{ self.header[self.header_idx]=b; self.add_ck(b); self.header_idx+=1; if self.header_idx==4 { self.payload_len=((self.header[3] as u16)<<8) | (self.header[2] as u16); if self.header[0]==0x01 && self.header[1]==0x07 && self.payload_len==92 { self.state=ParserState::Payload; self.payload_idx=0; } else { self.reset(); } } }, ParserState::Payload=>{ self.payload[self.payload_idx]=b; self.add_ck(b); self.payload_idx+=1; if self.payload_idx as u16==self.payload_len { self.state=ParserState::Checksum; self.cidx=0; } }, ParserState::Checksum=>{ if self.cidx==0 { self.checksum_a=b; self.cidx=1; } else { self.checksum_b=b; if self.calc_a==self.checksum_a && self.calc_b==self.checksum_b { let r=self.parse_nav_pvt(); self.reset(); return r; } else { self.reset(); } } } } None }
    fn parse_nav_pvt(&self)->Option<GpsData>{ let p=&self.payload; let mut g=GpsData::default(); g.year=u16::from_le_bytes([p[4],p[5]]); g.month=p[6]; g.day=p[7]; g.hour=p[8]; g.minute=p[9]; g.second=p[10]; g.fix_type=p[20]; g.satellites=p[23]; g.longitude=i32::from_le_bytes([p[24],p[25],p[26],p[27]]); g.latitude=i32::from_le_bytes([p[28],p[29],p[30],p[31]]); g.altitude=i32::from_le_bytes([p[36],p[37],p[38],p[39]]); Some(g) }
}

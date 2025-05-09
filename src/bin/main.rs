#![no_std]
#![no_main]
#![feature(allocator_api)]

use core::alloc::Layout;

use alloc::{alloc::alloc_zeroed, vec};
use allocator_api2::{alloc::Allocator, boxed::Box, vec::Vec};
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Timer};
use esp_alloc::{EspHeap, HEAP};
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    dma::{DmaRxBuf, DmaTxBuf},
    gpio::{Level, Output},
    psram::{self, FlashFreq, PsramConfig, PsramSize},
    spi::{
        master::{Config, Spi, SpiDmaBus},
        Mode,
    },
    time::Rate,
    timer::systimer::SystemTimer,
    Async,
};
use mipidsi::{
    interface::SpiInterface,
    options::{HorizontalRefreshOrder, RefreshOrder, VerticalRefreshOrder},
    TestImage,
};
use mipidsi::{models::ST7789, options::ColorInversion, Builder};
use static_cell::StaticCell;
extern crate alloc;
use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{
        Circle, PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, StrokeAlignment, Styled, Triangle,
    },
    text::{Alignment, Text},
};
use embedded_graphics_framebuf::FrameBuf;
use log::{error, info};

static PSRAM_ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

// Display
// Size
const W: u16 = 205;
const H: u16 = 320;
// Offset
const X_OFFSET: u16 = 35;
const Y_OFFSET: u16 = 0;
// Active area
const W_ACTIVE: usize = (W - X_OFFSET) as usize; //170
const H_ACTIVE: usize = (H - Y_OFFSET) as usize; //320

const FULL_FRAME_SIZE: usize = W_ACTIVE as usize * H_ACTIVE as usize * 2;

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let mut psram_conf = PsramConfig::default();
    psram_conf.size = PsramSize::Size(8 * 1024 * 1024);
    let conf = esp_hal::Config::default()
        .with_cpu_clock(CpuClock::_240MHz)
        .with_psram(psram_conf);
    let p = esp_hal::init(conf);
    esp_alloc::heap_allocator!(#[unsafe(link_section = ".dram2_uninit")] size: 64000);
    esp_alloc::heap_allocator!(size: 150 * 1024);
    let (start, size) = psram::psram_raw_parts(&p.PSRAM);
    info!("PSRAM start: {}, size: {}", start as usize, size as usize);
    unsafe {
        PSRAM_ALLOCATOR.add_region(esp_alloc::HeapRegion::new(
            start,
            size,
            esp_alloc::MemoryCapability::External.into(),
        ));
    }

    let timer0 = SystemTimer::new(p.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = esp_hal::dma_buffers!(4, 32_000);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    // Creating SPI
    let sclk = p.GPIO4;
    let mosi = p.GPIO5;
    let res = p.GPIO6;
    let dc = p.GPIO7;
    let cs = p.GPIO15;
    let spi = Spi::new(
        p.SPI2,
        Config::default()
            .with_frequency(Rate::from_mhz(80))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(sclk)
    .with_mosi(mosi)
    .with_dma(p.DMA_CH0)
    .with_buffers(dma_rx_buf, dma_tx_buf)
    .into_async();

    let mut disp_buffer: Vec<u8, &EspHeap> = Vec::new_in(&HEAP);
    disp_buffer.resize(2048, 0);

    let res = Output::new(res, Level::Low, Default::default());
    let dc = Output::new(dc, Level::Low, Default::default());
    let cs = Output::new(cs, Level::High, Default::default());

    static SPI_BUS: StaticCell<Mutex<NoopRawMutex, SpiDmaBus<'static, Async>>> = StaticCell::new();
    let spi_bus = Mutex::new(spi);
    let spi_bus = SPI_BUS.init(spi_bus);
    let spi_device = SpiDevice::new(spi_bus, cs);

    let di = SpiInterface::new(spi_device, dc, &mut disp_buffer);
    let mut delay = embassy_time::Delay;
    let mut display = Builder::new(ST7789, di)
        .display_size(W_ACTIVE as u16, H_ACTIVE as u16)
        .display_offset(X_OFFSET, Y_OFFSET)
        .refresh_order(RefreshOrder {
            vertical: VerticalRefreshOrder::BottomToTop,
            horizontal: HorizontalRefreshOrder::RightToLeft,
        })
        .invert_colors(ColorInversion::Inverted)
        .reset_pin(res)
        .init(&mut delay)
        .await
        .unwrap();
    info!("Display initialized!");

    let mut data: Vec<Rgb565, &EspHeap> = Vec::new_in(&PSRAM_ALLOCATOR);
    data.resize(FULL_FRAME_SIZE / 2, Rgb565::BLACK);
    {
        let mut fbuf = FrameBuf::new(data.as_mut_slice(), W_ACTIVE, H_ACTIVE);
        Rectangle {
            top_left: Point { x: 0, y: 0 },
            size: Size {
                width: W_ACTIVE as u32,
                height: H_ACTIVE as u32,
            },
        }
        .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
        .draw(&mut fbuf)
        .unwrap();
    }
    display
        .set_pixels(0, 0, W_ACTIVE as u16 - 1, H_ACTIVE as u16 - 1, data)
        .await
        .unwrap();

    Timer::after(Duration::from_secs(1)).await;


    let mut data2: Vec<Rgb565, &EspHeap> = Vec::new_in(&PSRAM_ALLOCATOR);
    data2.resize(FULL_FRAME_SIZE / 2, Rgb565::BLACK);
    let mut fbuf2 = FrameBuf::new(data2.as_mut_slice(), W_ACTIVE, H_ACTIVE);

    let start = Instant::now();
    TestImage::new().draw(&mut fbuf2).unwrap();
    let elapsed = start.elapsed();
    info!("Drew image in {}us", elapsed.as_micros());

    let start = Instant::now();
    display
        .set_pixels(0, 0, W_ACTIVE as u16 - 1, H_ACTIVE as u16 - 1, data2)
        .await
        .unwrap();
    let elapsed = start.elapsed();
    info!("Sent in {}us", elapsed.as_micros());

    info!("Global heap stats: {}", HEAP.stats());
    info!("PSRAM heap stats: {}", PSRAM_ALLOCATOR.stats());

    loop {
        info!("Hello world!");
        Timer::after(Duration::from_secs(3)).await;
    }
}

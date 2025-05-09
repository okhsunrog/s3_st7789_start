#![no_std]
#![no_main]
#![feature(allocator_api)]

use core::alloc::Layout;

use alloc::{alloc::alloc_zeroed, vec};
use allocator_api2::{alloc::Allocator, boxed::Box, vec::Vec};
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Duration, Instant, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_alloc::{EspHeap, HEAP};
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    dma::{DmaRxBuf, DmaTxBuf, ExternalBurstConfig},
    gpio::{Level, Output},
    peripherals::DMA,
    psram::{self, FlashFreq, PsramConfig, PsramSize, SpiRamFreq, SpiTimingConfigCoreClock},
    spi::{
        master::{Config, Spi, SpiDmaBus},
        Mode,
    },
    time::Rate,
    timer::{systimer::SystemTimer, timg::TimerGroup},
    Async,
};
use mipidsi::{
    interface::SpiInterface,
    options::{HorizontalRefreshOrder, RefreshOrder, VerticalRefreshOrder},
    TestImage,
};
use mipidsi::{models::ST7789, options::ColorInversion, Builder};
extern crate alloc;
use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{
        Circle, PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, StrokeAlignment, Triangle,
    },
    text::{Alignment, Text},
};
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
const W_ACTIVE: u16 = W - X_OFFSET; //170
const H_ACTIVE: u16 = H - Y_OFFSET; //320

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
    .with_buffers(dma_rx_buf, dma_tx_buf);

    let mut disp_buffer: Vec<u8, &EspHeap> = Vec::new_in(&HEAP);
    disp_buffer.resize(2048, 0);

    let res = Output::new(res, Level::Low, Default::default());
    let dc = Output::new(dc, Level::Low, Default::default());
    let cs = Output::new(cs, Level::High, Default::default());
    let spi_device = ExclusiveDevice::new_no_delay(spi, cs).unwrap();
    let di = SpiInterface::new(spi_device, dc, &mut disp_buffer);
    let mut delay = embassy_time::Delay;
    let mut display = Builder::new(ST7789, di)
        .display_size(W_ACTIVE, H_ACTIVE)
        .display_offset(X_OFFSET, Y_OFFSET)
        .refresh_order(RefreshOrder {
            vertical: VerticalRefreshOrder::BottomToTop,
            horizontal: HorizontalRefreshOrder::RightToLeft,
        })
        .invert_colors(ColorInversion::Inverted)
        .reset_pin(res)
        .init(&mut delay)
        .unwrap();
    info!("Display initialized!");

    let start = Instant::now();
    display
        .fill_solid(
            &Rectangle {
                top_left: Point { x: 0, y: 0 },
                size: Size {
                    width: W_ACTIVE as u32,
                    height: H_ACTIVE as u32,
                },
            },
            Rgb565::BLACK,
        )
        .unwrap();
    let elapsed = start.elapsed();
    info!("Filled and sent in {}us", elapsed.as_micros());
    Timer::after(Duration::from_secs(1)).await;

    let start = Instant::now();
    TestImage::new().draw(&mut display).unwrap();
    let elapsed = start.elapsed();
    info!("Drew image in {}us", elapsed.as_micros());

    info!("Global heap stats: {}", HEAP.stats());
    info!("PSRAM heap stats: {}", PSRAM_ALLOCATOR.stats());

    loop {
        info!("Hello world!");
        Timer::after(Duration::from_secs(3)).await;
    }
}

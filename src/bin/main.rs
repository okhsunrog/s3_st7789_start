#![no_std]
#![no_main]
#![feature(allocator_api)]

use alloc::format;
use allocator_api2::vec::Vec;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::Instant;
use esp_alloc::{EspHeap, HEAP};
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    dma::{DmaRxBuf, DmaTxBuf},
    gpio::{Level, Output},
    psram::{self, PsramConfig, PsramSize},
    spi::{
        master::{Config, Spi, SpiDmaBus},
        Mode,
    },
    time::Rate,
    timer::systimer::SystemTimer,
    Async,
};

use mipidsi::{interface::SpiInterface, models::ST7789, options::ColorInversion, raw_framebuf::RawFrameBuf, Builder};

use static_cell::StaticCell;
extern crate alloc;
use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::{Alignment, Text},
};
use log::info;

static PSRAM_ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

const W: u16 = 205;
const H: u16 = 320;
const X_OFFSET: u16 = 35;
const Y_OFFSET: u16 = 0;
const W_ACTIVE: usize = (W - X_OFFSET) as usize; // 170
const H_ACTIVE: usize = (H - Y_OFFSET) as usize; // 320
const PXL_SIZE: usize = 2;
const FRAME_BYTE_SIZE: usize = W_ACTIVE * H_ACTIVE * PXL_SIZE;
const RECT_WIDTH: u32 = 40;
const RECT_HEIGHT: u32 = 30;
const RECT_SPEED: i32 = 3;

struct AnimationState {
    rect_x: i32,
    direction: i32,
    fps: u32,
}

fn update_animation_state(state: &mut AnimationState) {
    state.rect_x += state.direction * RECT_SPEED;
    if state.rect_x <= 0 {
        state.rect_x = 0;
        state.direction = 1;
    } else if state.rect_x as u32 + RECT_WIDTH >= W_ACTIVE as u32 {
        state.rect_x = (W_ACTIVE as u32 - RECT_WIDTH) as i32;
        state.direction = -1;
    }
}

// - bench
// 	- 170x320, rectangle, internal buffer (PSRAM was activates)
// 		- Drew image in 883us
// 		- Sent in 11429us
// 	- 170x320, rectangle, internal buffer (PSRAM was not activates)
// 		- Drew image in 796us
// 		- Sent in 11431us / 11409us (oscillates, in between)
// 	- 170x320, rectangle, buffer in PSRAM
// 		- Drew image in 5844us
// 		- Sent in 15167us
// -
// - drawing in PSRAM is expensive!
// - hmm, just enabling PSRAM allocator changes fps from 83 to 82

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let mut psram_conf = PsramConfig::default();
    psram_conf.size = PsramSize::Size(8 * 1024 * 1024);
    let conf = esp_hal::Config::default()
        .with_cpu_clock(CpuClock::_240MHz)
        .with_psram(psram_conf);
    let p = esp_hal::init(conf);
    // esp_alloc::heap_allocator!(#[unsafe(link_section = ".dram2_uninit")] size: 64000);
    esp_alloc::heap_allocator!(size: 150 * 1024); // 250K seems to work too
    let (start, size) = psram::psram_raw_parts(&p.PSRAM);
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
        Config::default().with_frequency(Rate::from_mhz(80)).with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(sclk)
    .with_mosi(mosi)
    .with_dma(p.DMA_CH0)
    .with_buffers(dma_rx_buf, dma_tx_buf)
    .into_async();

    let res = Output::new(res, Level::Low, Default::default());
    let dc = Output::new(dc, Level::Low, Default::default());
    let cs = Output::new(cs, Level::High, Default::default());

    static SPI_BUS: StaticCell<Mutex<NoopRawMutex, SpiDmaBus<'static, Async>>> = StaticCell::new();
    let spi_bus = Mutex::new(spi);
    let spi_bus = SPI_BUS.init(spi_bus);
    let spi_device = SpiDevice::new(spi_bus, cs);

    let di = SpiInterface::new(spi_device, dc);
    let mut delay = embassy_time::Delay;

    let mut display = Builder::new(ST7789, di)
        .reset_pin(res)
        .display_size(W_ACTIVE, H_ACTIVE)
        .display_offset(X_OFFSET, Y_OFFSET)
        .invert_colors(ColorInversion::Inverted)
        .init(&mut delay)
        .await
        .unwrap();
    info!("Display initialized!");

    let mut animation_state = AnimationState {
        rect_x: 0,
        direction: 1,
        fps: 0,
    };
    let mut fps_counter = 0;
    let mut last_fps_update = Instant::now();

    let mut frame: Vec<u8, &EspHeap> = Vec::new_in(&PSRAM_ALLOCATOR);

    frame.resize(FRAME_BYTE_SIZE, 0);
    info!("Global heap stats: {}", HEAP.stats());
    info!("PSRAM heap stats: {}", PSRAM_ALLOCATOR.stats());

    loop {
        {
            let start = Instant::now();
            let mut raw_fb = RawFrameBuf::<Rgb565, _, PXL_SIZE>::new(frame.as_mut_slice(), W_ACTIVE, H_ACTIVE);
            raw_fb.clear(Rgb565::BLACK).unwrap();
            Rectangle::new(
                Point::new(animation_state.rect_x, 70),
                Size::new(RECT_WIDTH, RECT_HEIGHT),
            )
            .into_styled(PrimitiveStyle::with_fill(Rgb565::RED))
            .draw(&mut raw_fb)
            .unwrap();
            let fps_text = format!("FPS: {}", animation_state.fps);
            Text::with_alignment(
                &fps_text,
                Point::new(W_ACTIVE as i32 / 2, H_ACTIVE as i32 - 20),
                MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE),
                Alignment::Center,
            )
            .draw(&mut raw_fb)
            .unwrap();
            let elapsed = start.elapsed();
            info!("Drew image in {}us", elapsed.as_micros());
        }
        update_animation_state(&mut animation_state);

        let start = Instant::now();
        display.show_raw_data(0, 0, W_ACTIVE, H_ACTIVE, &frame).await.unwrap();
        let elapsed = start.elapsed();
        info!("Sent in {}us", elapsed.as_micros());

        fps_counter += 1;
        if last_fps_update.elapsed().as_millis() >= 1000 {
            animation_state.fps = fps_counter;
            fps_counter = 0;
            last_fps_update = Instant::now();
        }
    }
}

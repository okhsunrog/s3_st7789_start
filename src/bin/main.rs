#![no_std]
#![no_main]
#![feature(allocator_api)]

use core::alloc::Layout;

use alloc::{alloc::alloc_zeroed, vec, format};
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
const FULL_FRAME_SIZE: usize = W_ACTIVE * H_ACTIVE * 2;

// Animation constants
const RECT_WIDTH: u32 = 40;
const RECT_HEIGHT: u32 = 30;
const RECT_SPEED: i32 = 3;

// Animation state struct
struct AnimationState {
    rect_x: i32,
    direction: i32,
    fps: u32,
}

// Function to update animation state
fn update_animation_state(state: &mut AnimationState) {
    // Update rectangle position
    state.rect_x += state.direction * RECT_SPEED;

    // Check for bouncing
    if state.rect_x <= 0 {
        state.rect_x = 0;
        state.direction = 1;
    } else if state.rect_x as u32 + RECT_WIDTH >= W_ACTIVE as u32 {
        state.rect_x = (W_ACTIVE as u32 - RECT_WIDTH) as i32;
        state.direction = -1;
    }
}

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
        .display_size(W_ACTIVE as u16, H_ACTIVE as u16)
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
    
    // Initialize animation state
    let mut animation_state = AnimationState {
        rect_x: 0,
        direction: 1, // 1 = right, -1 = left
        fps: 0,
    };

    // FPS tracking variables
    let mut fps_counter = 0;
    let mut last_fps_update = Instant::now();
    let mut frame_time_sum = 0u64;
    let mut frame_count = 0;

    // Initial black screen
    let mut data: Vec<Rgb565, &EspHeap> = Vec::new_in(&PSRAM_ALLOCATOR);
    data.resize(W_ACTIVE * H_ACTIVE, Rgb565::BLACK);
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
        .unwrap();

    // Animation loop
    loop {
        let frame_start = Instant::now();
        
        // Create a new frame buffer for this frame
        
        
        // Draw to the frame buffer
        {
            let mut frame_data: Vec<Rgb565, &EspHeap> = Vec::new_in(&HEAP);
            frame_data.resize(W_ACTIVE * H_ACTIVE, Rgb565::BLACK);
            let mut fbuf = FrameBuf::new(frame_data.as_mut_slice(), W_ACTIVE, H_ACTIVE);
            
            // Clear with black
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
            
            // Draw the moving rectangle
            Rectangle {
                top_left: Point { x: animation_state.rect_x, y: 70 },
                size: Size {
                    width: RECT_WIDTH,
                    height: RECT_HEIGHT,
                },
            }
            .into_styled(PrimitiveStyle::with_fill(Rgb565::RED))
            .draw(&mut fbuf)
            .unwrap();
            
            // Draw FPS text
            let fps_text = format!("FPS: {}", animation_state.fps);
            let text_style = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);
            
            Text::with_alignment(
                &fps_text,
                Point::new(W_ACTIVE as i32 / 2, H_ACTIVE as i32 - 20),
                text_style,
                Alignment::Center,
            )
            .draw(&mut fbuf)
            .unwrap();

             // Update animation state for next frame
             update_animation_state(&mut animation_state);

             // Send the entire framebuffer to display at once
             display
                 .set_pixels(
                     0, 
                     0, 
                     W_ACTIVE as u16 - 1, 
                     H_ACTIVE as u16 - 1, 
                     frame_data
                 )
                 .unwrap();
        }
        
   

        // Update FPS counter
        fps_counter += 1;
        
        // Calculate frame time for statistics
        let frame_time = frame_start.elapsed();
        frame_time_sum += frame_time.as_micros() as u64;
        frame_count += 1;
        
        if last_fps_update.elapsed().as_millis() >= 1000 {
            animation_state.fps = fps_counter;
            fps_counter = 0;
            
            // Log FPS and average frame time
            //let avg_frame_time = if frame_count > 0 { frame_time_sum / frame_count } else { 0 };
            //info!("Current FPS: {}, Avg frame time: {}µs", animation_state.fps, avg_frame_time);
            
            // Reset statistics
            frame_time_sum = 0;
            frame_count = 0;
            last_fps_update = Instant::now();
        }
    }
}

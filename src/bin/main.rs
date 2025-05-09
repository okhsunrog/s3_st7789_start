#![no_std]
#![no_main]
#![feature(allocator_api)]

use alloc::{format, vec};
use allocator_api2::{alloc::Allocator, vec::Vec};
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Ticker};
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
use mipidsi::{
    interface::SpiInterface,
    models::ST7789,
    options::{ColorInversion, HorizontalRefreshOrder, RefreshOrder, VerticalRefreshOrder},
    Builder,
};
use static_cell::StaticCell;
extern crate alloc;
use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::{Alignment, Text},
};
use embedded_graphics_framebuf::FrameBuf;
use log::info;

static PSRAM_ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();
static mut SMALL_BUFFER: [u8; 2048] = [0u8; 2048];

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

// Define a custom iterator that yields pixels from our frame buffer
struct FrameBufferIterator<'a> {
    buffer: &'a [Rgb565],
    index: usize,
}

impl<'a> FrameBufferIterator<'a> {
    fn new(buffer: &'a [Rgb565]) -> Self {
        Self {
            buffer,
            index: 0,
        }
    }
}

impl<'a> Iterator for FrameBufferIterator<'a> {
    type Item = Rgb565;

    fn next(&mut self) -> Option<Self::Item> {
        if self.index < self.buffer.len() {
            let pixel = self.buffer[self.index];
            self.index += 1;
            Some(pixel)
        } else {
            None
        }
    }
}

// Define our frame type
type Frame = Vec<Rgb565, &'static EspHeap>;

// Signals for frame synchronization - using CriticalSectionRawMutex which is Sync
static NEXT_FRAME: Signal<CriticalSectionRawMutex, Frame> = Signal::new();
static READY_FRAME: Signal<CriticalSectionRawMutex, Frame> = Signal::new();

// Display type alias
type DisplayType = mipidsi::Display<
    SpiInterface<'static,
        SpiDevice<'static, NoopRawMutex, SpiDmaBus<'static, Async>, Output<'static>>,
        Output<'static>
    >,
    ST7789,
    Output<'static>
>;


// Display task that handles sending frames to the display
#[embassy_executor::task]
async fn display_task(mut display: DisplayType) {
    let mut frame = READY_FRAME.wait().await;
    
    loop {
        // Send the frame to the display
        display
            .set_pixels(
                0, 
                0, 
                W_ACTIVE as u16 - 1, 
                H_ACTIVE as u16 - 1, 
                FrameBufferIterator::new(&frame)
            )
            .await
            .unwrap();
        
        // Signal that we're ready for the next frame and wait for it
        NEXT_FRAME.signal(frame);
        frame = READY_FRAME.wait().await;
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let mut psram_conf = PsramConfig::default();
    psram_conf.size = PsramSize::Size(8 * 1024 * 1024);
    let conf = esp_hal::Config::default()
        .with_cpu_clock(CpuClock::_240MHz)
        .with_psram(psram_conf);
    let p = esp_hal::init(conf);
    esp_alloc::heap_allocator!(#[unsafe(link_section = ".dram2_uninit")] size: 64000);
    esp_alloc::heap_allocator!(size: 250 * 1024);

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

    // Small buffer for SpiInterface - we only need a minimal buffer since we're not using it for drawing
    let small_buffer = unsafe { &mut SMALL_BUFFER };

    let res = Output::new(res, Level::Low, Default::default());
    let dc = Output::new(dc, Level::Low, Default::default());
    let cs = Output::new(cs, Level::High, Default::default());
    
    static SPI_BUS: StaticCell<Mutex<NoopRawMutex, SpiDmaBus<'static, Async>>> = StaticCell::new();
    let spi_bus = Mutex::new(spi);
    let spi_bus = SPI_BUS.init(spi_bus);
    let spi_device = SpiDevice::new(spi_bus, cs);
    
    let di = SpiInterface::new(spi_device, dc, small_buffer);
    let mut delay = embassy_time::Delay;
    let display = Builder::new(ST7789, di)
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

    // Initialize animation state
    let mut animation_state = AnimationState {
        rect_x: 0,
        direction: 1, // 1 = right, -1 = left
        fps: 0,
    };

    // FPS tracking variables
    let mut fps_counter = 0;
    let mut last_fps_update = Instant::now();

    // Create two frame buffers for double buffering
    // You can choose to allocate these in PSRAM or internal memory based on your performance needs
    let mut frame_a: Frame = Vec::new_in(&HEAP);
    let mut frame_b: Frame = Vec::new_in(&HEAP);
    
    frame_a.resize(W_ACTIVE * H_ACTIVE, Rgb565::BLACK);
    frame_b.resize(W_ACTIVE * H_ACTIVE, Rgb565::BLACK);

    info!("Global heap stats: {}", HEAP.stats());
    info!("PSRAM heap stats: {}", PSRAM_ALLOCATOR.stats());

    // Initialize the first frame with black
    {
        let mut fbuf = FrameBuf::new(frame_a.as_mut_slice(), W_ACTIVE, H_ACTIVE);
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

    // Signal the first frame as ready
    NEXT_FRAME.signal(frame_a);
    
    // Spawn the display task
    spawner.spawn(display_task(display)).unwrap();
    
    // Main rendering loop
    let mut ticker = Ticker::every(Duration::from_millis(1));
    
    loop {
        // Wait for the next frame buffer to be available
        let mut frame = NEXT_FRAME.wait().await;
        
        // Draw to the frame
        {
            let mut fbuf = FrameBuf::new(frame.as_mut_slice(), W_ACTIVE, H_ACTIVE);
            
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
        }
        
        // Update animation state for next frame
        update_animation_state(&mut animation_state);
        
        // Signal that the frame is ready for display
        READY_FRAME.signal(frame);
        
        // Update FPS counter
        fps_counter += 1;
        
        if last_fps_update.elapsed().as_millis() >= 1000 {
            animation_state.fps = fps_counter;
            fps_counter = 0;
            last_fps_update = Instant::now();
        }
        
        // Pace the rendering
        ticker.next().await;
    }
}

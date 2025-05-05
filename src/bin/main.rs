#![no_std]
#![no_main]
#![feature(allocator_api)]

use core::alloc::Layout;

use allocator_api2::{alloc::Allocator, vec::Vec};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_alloc::HEAP;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock, dma::{DmaRxBuf, DmaTxBuf, ExternalBurstConfig}, gpio::{Level, Output}, peripherals::DMA, psram::{self, FlashFreq, PsramConfig, PsramSize, SpiRamFreq, SpiTimingConfigCoreClock}, spi::{
        master::{Config, Spi, SpiDmaBus}, Mode
    }, time::Rate, timer::{systimer::SystemTimer, timg::TimerGroup}, Async
};
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use static_cell::StaticCell;
use embassy_sync::mutex::Mutex;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use mipidsi::interface::SpiInterface;
use mipidsi::{models::ST7789, options::ColorInversion, Builder};
extern crate alloc;
use log::{error, info};

static PSRAM_ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

const DMA_BUFFER_SIZE: usize = 32000;
const DMA_ALIGNMENT: ExternalBurstConfig = ExternalBurstConfig::Size64;
const DMA_CHUNK_SIZE: usize = 4096 - DMA_ALIGNMENT as usize;

// Display
const W: i32 = 320;
const H: i32 = 170;

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default()
        .with_cpu_clock(CpuClock::max())
        .with_psram(PsramConfig {
            size: PsramSize::Size(2097152),
            core_clock: SpiTimingConfigCoreClock::SpiTimingConfigCoreClock80m,
            flash_frequency: FlashFreq::FlashFreq80m,
            ram_frequency: SpiRamFreq::Freq40m,
        });

    let p = esp_hal::init(config);
    esp_alloc::heap_allocator!(size: 72 * 1024);
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

    info!("Embassy initialized!");

    let timer1 = TimerGroup::new(p.TIMG0);
    let _init = esp_wifi::init(timer1.timer0, esp_hal::rng::Rng::new(p.RNG), p.RADIO_CLK).unwrap();

    // DMA for SPI
    let (_, tx_descriptors) =
        esp_hal::dma_descriptors_chunk_size!(0, DMA_BUFFER_SIZE, DMA_CHUNK_SIZE);
    let layout =
        core::alloc::Layout::from_size_align(DMA_BUFFER_SIZE, DMA_ALIGNMENT as usize).unwrap();
    let tx_buffer: &mut [u8] = unsafe { PSRAM_ALLOCATOR.allocate(layout).unwrap().as_mut() };
    info!(
        "TX: {:p} len {} ({} descripters)",
        tx_buffer.as_ptr(),
        tx_buffer.len(),
        tx_descriptors.len()
    );
    let dma_tx_buf =
        DmaTxBuf::new_with_config(tx_descriptors, tx_buffer, DMA_ALIGNMENT).unwrap();
    let (rx_buffer, rx_descriptors, _, _) = esp_hal::dma_buffers!(4, 0);
    info!(
        "RX: {:p} len {} ({} descripters)",
        rx_buffer.as_ptr(),
        rx_buffer.len(),
        rx_descriptors.len()
    );
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

    // Creating SPI
    let sclk = p.GPIO7;
    let mosi = p.GPIO6;
    let res = p.GPIO5;
    let dc = p.GPIO4;
    let cs = p.GPIO2;

    // Need to set miso first so that mosi can overwrite the
    // output connection (because we are using the same pin to loop back)
    let spi = Spi::new(
        p.SPI2,
        Config::default()
            .with_frequency(Rate::from_khz(100))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(sclk)
    .with_mosi(mosi)
    .with_dma(p.DMA_CH0)
    .with_buffers(dma_rx_buf, dma_tx_buf)
    .into_async();

    let disp_buffer = unsafe {PSRAM_ALLOCATOR.allocate(Layout::from_size_align(1024, 1).unwrap()).unwrap().as_mut()};
    let res = Output::new(res, Level::Low, Default::default());
    let dc = Output::new(dc, Level::Low, Default::default());
    let cs = Output::new(cs, Level::High, Default::default());
    static SPI_BUS: StaticCell<Mutex<NoopRawMutex, SpiDmaBus<'static, Async>>> = StaticCell::new();
    let spi_bus = Mutex::new(spi);
    let spi_bus = SPI_BUS.init(spi_bus);
    let spi_device = SpiDevice::new(spi_bus, cs);
    let di = SpiInterface::new(spi_device, dc, disp_buffer);
    let mut delay = embassy_time::Delay;
    let display = Builder::new(ST7789, di).display_size(W as u16, H as u16)
    .invert_colors(ColorInversion::Inverted)
    .reset_pin(res)
    .init(&mut delay)
    .await
    .unwrap();

    

    // TODO: Spawn some tasks
    let _ = spawner;

    info!("Global heap stats: {}", HEAP.stats());
    info!("PSRAM heap stats: {}", PSRAM_ALLOCATOR.stats());

    loop {
        info!("Hello world!");
        Timer::after(Duration::from_secs(1)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-beta.0/examples/src/bin
}

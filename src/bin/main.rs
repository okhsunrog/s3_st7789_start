#![no_std]
#![no_main]
#![feature(allocator_api)]

use allocator_api2::{alloc::Allocator, vec::Vec};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_alloc::HEAP;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    dma::{DmaRxBuf, DmaTxBuf, ExternalBurstConfig},
    peripherals::DMA,
    spi::{
        Mode,
        master::{Config, Spi},
    },
    time::Rate,
    psram::{self, FlashFreq, PsramConfig, PsramSize, SpiRamFreq, SpiTimingConfigCoreClock},
    timer::{systimer::SystemTimer, timg::TimerGroup},
};
extern crate alloc;
use log::{error, info};

static PSRAM_ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

const DMA_BUFFER_SIZE: usize = 8192;
const DMA_ALIGNMENT: ExternalBurstConfig = ExternalBurstConfig::Size64;
const DMA_CHUNK_SIZE: usize = 4096 - DMA_ALIGNMENT as usize;

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
    let (rx_buffer, rx_descriptors, _, _) = esp_hal::dma_buffers!(DMA_BUFFER_SIZE, 0);
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
    let mut spi = Spi::new(
        p.SPI2,
        Config::default()
            .with_frequency(Rate::from_khz(100))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(sclk)
    .with_mosi(mosi)
    .with_dma(p.DMA_CH0)
    .with_buffers(dma_rx_buf, dma_tx_buf);
    

    // TODO: Spawn some tasks
    let _ = spawner;

    // Testing heaps
    let large_buffer: Vec<u8, _> = Vec::with_capacity_in(1048576, &PSRAM_ALLOCATOR);
    let small_buffer: Vec<u8, _> = Vec::with_capacity_in(1024, &HEAP);
    info!("Global heap stats: {}", HEAP.stats());
    info!("PSRAM heap stats: {}", PSRAM_ALLOCATOR.stats());

    loop {
        info!("Hello world!");
        Timer::after(Duration::from_secs(1)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-beta.0/examples/src/bin
}

#![no_std]
#![no_main]
#![feature(allocator_api)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_alloc::HEAP;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::psram::{self, FlashFreq, PsramConfig, PsramSize, SpiRamFreq, SpiTimingConfigCoreClock};
use esp_hal::timer::systimer::SystemTimer;
use esp_hal::timer::timg::TimerGroup;
use log::info;
use allocator_api2::vec::Vec;

extern crate alloc;

static PSRAM_ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

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
    let _init = esp_wifi::init(
        timer1.timer0,
        esp_hal::rng::Rng::new(p.RNG),
        p.RADIO_CLK,
    )
    .unwrap();

    // TODO: Spawn some tasks
    let _ = spawner;

    // Testing heaps
    let large_buffer: Vec<u8, _> = Vec::with_capacity_in(1048576, &PSRAM_ALLOCATOR);
    let small_buffer: Vec<u8, _> = Vec::with_capacity_in(1024, &HEAP);

    loop {
        info!("Hello world!");
        Timer::after(Duration::from_secs(1)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-beta.0/examples/src/bin
}

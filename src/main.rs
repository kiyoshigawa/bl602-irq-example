// This code will toggle the state of the the channels of the RGB LED on the Pinecone eval board
// using timer-based interrupts as well as a normal delay loop.

#![no_std]
#![no_main]

use bl602_hal as hal;
use core::cell::RefCell;
use core::ops::DerefMut;
use embedded_hal::delay::blocking::DelayMs;
use embedded_hal::digital::blocking::{OutputPin, ToggleableOutputPin};
use embedded_time::duration::Milliseconds;
use hal::{
    clock::{Strict, SysclkFreq},
    gpio::{Output, PullDown},
    interrupts::*,
    pac,
    prelude::*,
    timer::*,
};
use panic_halt as _;
use riscv::interrupt::Mutex;

// Setup custom types to make the code  below easier to read:
type BlueLedPin = hal::gpio::Pin11<Output<PullDown>>;
type LedTimer = hal::timer::ConfiguredTimerChannel0;

// Initialize global static containers:
static G_INTERRUPT_LED_PIN: Mutex<RefCell<Option<BlueLedPin>>> = Mutex::new(RefCell::new(None));
static G_LED_TIMER: Mutex<RefCell<Option<LedTimer>>> = Mutex::new(RefCell::new(None));

#[riscv_rt::entry]
fn main() -> ! {
    // Setup the device peripherals:
    let dp = pac::Peripherals::take().unwrap();
    let mut glb = dp.GLB.split();

    // Set up all the clocks we need:
    let clocks = Strict::new()
        .use_pll(40_000_000u32.Hz())
        .sys_clk(SysclkFreq::Pll160Mhz)
        .freeze(&mut glb.clk_cfg);

    // Initialize all led pins to their off state:
    let mut red_led_pin = glb.pin17.into_pull_down_output();
    let _ = red_led_pin.set_high();

    let mut blue_led_pin = glb.pin11.into_pull_down_output();
    let _ = blue_led_pin.set_low();

    // Set up the timer interrupt on TimerCh0 using the Match0 register:
    let timers = dp.TIMER.split();
    let timer_ch0 = timers
        .channel0
        .set_clock_source(ClockSource::Clock1Khz, 1_000_u32.Hz());
    timer_ch0.enable_match0_interrupt();
    timer_ch0.set_preload_value(Milliseconds::new(0));
    timer_ch0.set_preload(hal::timer::Preload::PreloadMatchComparator0);
    timer_ch0.set_match0(Milliseconds::new(500_u32));
    timer_ch0.enable();

    // Move the references to their UnsafeCells once initialized, and before interrupts are enabled.
    riscv::interrupt::free(|cs| G_INTERRUPT_LED_PIN.borrow(cs).replace(Some(blue_led_pin)));
    riscv::interrupt::free(|cs| G_LED_TIMER.borrow(cs).replace(Some(timer_ch0)));

    // Enable the timer interrupt only after pin and timer setup and move to global references:
    enable_interrupt(Interrupt::TimerCh0);

    // Create a blocking delay function based on the current cpu frequency
    let mut d = bl602_hal::delay::McycleDelay::new(clocks.sysclk().0);

    loop {
        // The loop blinks the red channel to show it is still runninng in the background:
        let _ = d.delay_ms(2000);
        let _ = red_led_pin.toggle();
    }
}

#[allow(non_snake_case)]
#[no_mangle]
fn TimerCh0(_trap_frame: &mut TrapFrame) {
    disable_interrupt(Interrupt::TimerCh0);
    clear_interrupt(Interrupt::TimerCh0);

    //clear the timer match0 interrupt:
    riscv::interrupt::free(|cs| {
        if let Some(timer) = G_LED_TIMER.borrow(cs).borrow_mut().deref_mut() {
            timer.clear_match0_interrupt();
        }
    });

    //Get and toggle the led pin:
    riscv::interrupt::free(|cs| {
        if let Some(led_pin) = G_INTERRUPT_LED_PIN.borrow(cs).borrow_mut().deref_mut() {
            led_pin.toggle().ok();
        }
    });

    // Re-enable interrupt when done:
    enable_interrupt(Interrupt::TimerCh0);
}

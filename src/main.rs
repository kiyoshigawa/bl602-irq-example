// This code will toggle the state of the the channels of the RGB LED on the Pinecone eval board
// using timer-based interrupts as well as a normal delay loop.

#![no_std]
#![no_main]

use bl602_hal as hal;
use core::mem::MaybeUninit;
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

// Setup custom types to make the code  below easier to read:
type BlueLedPin = hal::gpio::Pin11<Output<PullDown>>;
type LedTimer = hal::timer::ConfiguredTimerChannel0;

// Initialize global static containers and getters:
static mut G_INTERRUPT_LED_PIN: MaybeUninit<BlueLedPin> = MaybeUninit::uninit();
static mut G_LED_TIMER: MaybeUninit<LedTimer> = MaybeUninit::uninit();

fn get_interrupt_led_pin() -> &'static mut BlueLedPin {
    unsafe { &mut *G_INTERRUPT_LED_PIN.as_mut_ptr() }
}

fn get_led_timer() -> &'static mut LedTimer {
    unsafe { &mut *G_LED_TIMER.as_mut_ptr() }
}

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
    unsafe {
        *(G_INTERRUPT_LED_PIN.as_mut_ptr()) = blue_led_pin;
        *(G_LED_TIMER.as_mut_ptr()) = timer_ch0;
    }

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
    // Disable and clear the timer interrupts for the duration of this interrupt function:
    disable_interrupt(Interrupt::TimerCh0);
    clear_interrupt(Interrupt::TimerCh0);
    get_led_timer().disable();
    get_led_timer().clear_match0_interrupt();

    //Get and toggle the led pin:
    let _ = get_interrupt_led_pin().toggle();

    // Re-enable interrupts when done:
    get_led_timer().enable();
    enable_interrupt(Interrupt::TimerCh0);
}

#![no_main]
#![cfg_attr(not(test), no_std)]
#![deny(unsafe_code)]

extern crate panic_halt; // you can put a breakpoint on `rust_begin_unwind` to catch panics

use rtfm::app;
use stm32f1xx_hal::pac;
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::time::Hertz;
use stm32f1xx_hal::timer::Timer;

// The main frequency in Hz
const FREQUENCY: u32 = 48_000_000;

#[app(device = stm32f1::stm32f103, peripherals = true, monotonic = rtfm::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources { }

    /// Initialization happens here.
    ///
    /// The init function will run with interrupts disabled and has exclusive
    /// access to Cortex-M and device specific peripherals through the `core`
    /// and `device` variables, which are injected in the scope of init by the
    /// app attribute.
    #[init(spawn = [])]
    fn init(ctx: init::Context) {
        // Cortex-M peripherals
        let mut core: cortex_m::Peripherals = ctx.core;

        // Device specific peripherals
        let device: pac::Peripherals = ctx.device;

        // Get reference to peripherals
        let mut rcc = device.RCC.constrain();
        let mut afio = device.AFIO.constrain(&mut rcc.apb2);
        let gpioa = device.GPIOA.split(&mut rcc.apb2);
        let mut gpiob = device.GPIOB.split(&mut rcc.apb2);
        let mut flash = device.FLASH.constrain();

        // Disable JTAG to free up pins PA15, PB3 and PB4 for normal use
        let (_pa15, _pb3, _pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

        // Initialize (enable) the monotonic timer (CYCCNT)
        core.DCB.enable_trace();
        core.DWT.enable_cycle_counter();

        // Clock configuration
        let clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            .sysclk(Hertz(FREQUENCY))
            .pclk1(24.mhz())
            .freeze(&mut flash.acr);

        // Set up PWM timer on pin PB9
        let pin_pwm = gpiob.pb9.into_alternate_push_pull(&mut gpiob.crh);
        let mut pwm = Timer::tim4(device.TIM4, &clocks, &mut rcc.apb1)
            .pwm(pin_pwm, &mut afio.mapr, 50.hz());

        pwm.set_duty(pwm.get_max_duty() / 10);
        pwm.enable();
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        // The idle loop
        loop {}
    }

    // RTFM requires that free interrupts are declared in an extern block when
    // using software tasks; these free interrupts will be used to dispatch the
    // software tasks.
    extern "C" {
        fn SPI1();
        fn SPI2();
    }
};

#![no_main]
#![cfg_attr(not(test), no_std)]
#![deny(unsafe_code)]

extern crate panic_halt; // you can put a breakpoint on `rust_begin_unwind` to catch panics

use rtfm::app;
use rtfm::cyccnt::U32Ext;
use stm32f1xx_hal as hal;
use stm32f1xx_hal::adc::Adc;
use stm32f1xx_hal::gpio::{gpioa, gpiob, Alternate, Analog, PushPull};
use stm32f1xx_hal::pac;
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::pwm::Pwm;
use stm32f1xx_hal::rcc::Clocks;
use stm32f1xx_hal::time::Hertz;
use stm32f1xx_hal::timer::Timer;

// The main frequency in Hz
const FREQUENCY: u32 = 48_000_000;

// How often (in CPU cycles) the ADC should be polled
const POLL_PERIOD: u32 = FREQUENCY / 10; // ~100ms

pub struct Servo {
    pwm: Pwm<pac::TIM4, hal::pwm::C4>,
}

impl Servo {
    /// Create a new servo instance.
    pub fn new(
        pin: gpiob::PB9<Alternate<PushPull>>,
        tim: pac::TIM4,
        clocks: &Clocks,
        apb1: &mut hal::rcc::APB1,
        mapr: &mut hal::afio::MAPR,
    ) -> Self {
        let mut pwm = Timer::tim4(tim, &clocks, apb1).pwm(pin, mapr, 50.hz());
        pwm.set_duty(pwm.get_max_duty() / 200 * 15); // 1.5 ms by default, should turn off
        Self { pwm }
    }

    /// Enable the servo.
    pub fn enable(&mut self) {
        self.pwm.enable();
    }

    /// Disable the servo.
    pub fn disable(&mut self) {
        self.pwm.disable();
    }

    /// Set the speed from an ADC (12-bit) measurement.
    pub fn speed_from_adc(&mut self, value: u16) {
        // Clamp value
        let val = if value > 4095 {
            4095.0
        } else {
            value as f32
        };

        // Calculate servo PWM duty cycle:
        //    0 -> 1ms
        // 4096 -> 2ms
        let duty = (self.pwm.get_max_duty() as f32) / 20.0 * (1.0 + val / 4096.0);

        self.pwm.set_duty(duty as u16);
    }
}

#[app(device = stm32f1::stm32f103, peripherals = true, monotonic = rtfm::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        adc: Adc<pac::ADC1>,
        pin_adc: gpioa::PA1<Analog>,
        servo: Servo,
    }

    /// Initialization happens here.
    ///
    /// The init function will run with interrupts disabled and has exclusive
    /// access to Cortex-M and device specific peripherals through the `core`
    /// and `device` variables, which are injected in the scope of init by the
    /// app attribute.
    #[init(spawn = [poll_adc])]
    fn init(ctx: init::Context) -> init::LateResources {
        // Cortex-M peripherals
        let mut core: rtfm::Peripherals = ctx.core;

        // Device specific peripherals
        let device: pac::Peripherals = ctx.device;

        // Get reference to peripherals
        let mut rcc = device.RCC.constrain();
        let mut afio = device.AFIO.constrain(&mut rcc.apb2);
        let mut gpioa = device.GPIOA.split(&mut rcc.apb2);
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

        // Set up PA1 as ADC input pin connected to a potentiometer
        let pin_adc = gpioa.pa1.into_analog(&mut gpioa.crl);
        let mut adc = Adc::adc1(device.ADC1, &mut rcc.apb2, clocks);
        adc.default_cfg();

        // Schedule polling timer for ADC
        ctx.spawn.poll_adc().unwrap();

        // Set up PWM timer on pin PB9
        let pin_pwm = gpiob.pb9.into_alternate_push_pull(&mut gpiob.crh);
        let servo = Servo::new(pin_pwm, device.TIM4, &clocks, &mut rcc.apb1, &mut afio.mapr);

        // Assign resources
        init::LateResources {
            adc,
            pin_adc,
            servo,
        }
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        // The idle loop
        loop {}
    }

    /// Regularly called task that polls the ADC and adjusts PWM duty cycle.
    #[task(
        resources = [adc, pin_adc, servo],
        schedule = [poll_adc],
    )]
    fn poll_adc(ctx: poll_adc::Context) {
        // Read a 12-bit ADC value (0-4095).
        let value: u16 = ctx.resources.adc.read(ctx.resources.pin_adc).unwrap();

        // Adjust servo speed
        ctx.resources.servo.speed_from_adc(value);

        // Re-schedule the timer interrupt
        ctx.schedule
            .poll_adc(ctx.scheduled + POLL_PERIOD.cycles())
            .unwrap();
    }

    // RTFM requires that free interrupts are declared in an extern block when
    // using software tasks; these free interrupts will be used to dispatch the
    // software tasks.
    extern "C" {
        fn SPI1();
        fn SPI2();
    }
};

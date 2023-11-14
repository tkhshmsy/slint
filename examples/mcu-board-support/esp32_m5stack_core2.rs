use alloc::boxed::Box;
use alloc::rc::Rc;
use core::cell::RefCell;
use display_interface_spi::SPIInterface;
use embedded_graphics_core::geometry::OriginDimensions;
use embedded_hal::digital::OutputPin;
use embedded_hal_bus::i2c::RefCellDevice;
use esp_hal::clock::{ClockControl, CpuClock};
use esp_hal::delay::Delay;
use esp_hal::gpio::{Io, Level, Output};
use esp_hal::i2c::I2C;
use esp_hal::peripherals::{Peripherals, TIMG0, Interrupt};
use esp_hal::prelude::*;
use esp_hal::spi::{master::Spi, SpiMode};
use esp_hal::system::SystemControl;
use esp_hal::timer::timg::{Timer, Timer0, TimerGroup};
use esp_hal::interrupt::{self, Priority};
use esp_hal::rtc_cntl::Rtc;
pub use esp_hal::xtensa_lx_rt::entry;
use esp_alloc::EspHeap;
use esp_backtrace as _;
use mipidsi::Display;
use slint::platform::WindowEvent;
use dummy_pin::DummyPin;

use critical_section::Mutex;
static TIMER00: Mutex<RefCell<Option<Timer<Timer0<TIMG0>, esp_hal::Blocking>>>> = Mutex::new(RefCell::new(None));
static mut SYSTEM_COUNTER: u64 = 0;

#[global_allocator]
static ALLOCATOR: EspHeap = EspHeap::empty();

pub fn init() {
    const HEAP_SIZE: usize = 150 * 1024; // 150KB, original for CoreS3 is 200KB
    static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];
    unsafe { ALLOCATOR.init(core::ptr::addr_of_mut!(HEAP) as *mut u8, HEAP_SIZE) }
    slint::platform::set_platform(Box::new(EspBackend::default()))
        .expect("backend already initialized");
}

#[handler]
fn tg0_t0_level() {
    critical_section::with(|cs| {
        let mut timer = TIMER00.borrow_ref_mut(cs);
        let timer = timer.as_mut().unwrap();

        if timer.is_interrupt_set() {
            timer.clear_interrupt();
            timer.load_value(1u64.millis()).unwrap();
            timer.start();
            unsafe {
                core::ptr::write_volatile(core::ptr::addr_of_mut!(SYSTEM_COUNTER), SYSTEM_COUNTER.wrapping_add(1));
            }
        }
    });
}

#[derive(Default)]
struct EspBackend {
    window: RefCell<Option<Rc<slint::platform::software_renderer::MinimalSoftwareWindow>>>,
}

impl slint::platform::Platform for EspBackend {
    fn create_window_adapter(
        &self,
    ) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        let window = slint::platform::software_renderer::MinimalSoftwareWindow::new(
            slint::platform::software_renderer::RepaintBufferType::ReusedBuffer,
        );
        self.window.replace(Some(window.clone()));
        Ok(window)
    }

    fn duration_since_start(&self) -> core::time::Duration {
        core::time::Duration::from_millis(
            unsafe {
                core::ptr::read_volatile(core::ptr::addr_of_mut!(SYSTEM_COUNTER))
            }
        )
    }

    fn run_event_loop(&self) -> Result<(), slint::PlatformError> {
        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock240MHz).freeze();

        let mut rtc = Rtc::new(peripherals.LPWR, None);
        rtc.rwdt.disable();
        let mut timer_group0 =
            TimerGroup::new(peripherals.TIMG0, &clocks, None);
        timer_group0.wdt.disable();
        let mut timer_group1 =
            TimerGroup::new(peripherals.TIMG1, &clocks, None);
        timer_group1.wdt.disable();

        let timer00 = timer_group0.timer0;
        timer00.set_interrupt_handler(tg0_t0_level);
        interrupt::enable(Interrupt::TG0_T0_LEVEL, Priority::Priority2).unwrap();
        timer00.load_value(1u64.millis()).unwrap();
        timer00.start();
        timer00.listen();
        critical_section::with(|cs| {
            TIMER00.borrow_ref_mut(cs).replace(timer00);
        });

        let mut delay = Delay::new(&clocks);
        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

        let i2c = I2C::new(
            peripherals.I2C0,
            io.pins.gpio21, //sda
            io.pins.gpio22, //scl
            400u32.kHz(),
            &clocks,
            None
        );

        let i2c_bus = RefCell::new(i2c);
        let mut axp = axp192::Axp192::new(RefCellDevice::new(&i2c_bus));

        // core power
        axp.set_dcdc1_voltage(3350).unwrap();
        // LCD power
        axp.set_ldo2_voltage(3300).unwrap();
        axp.set_ldo2_on(true).unwrap();
        // LCD backlight
        axp.set_dcdc3_voltage(3300).unwrap();
        axp.set_dcdc3_on(true).unwrap();
        // LCD reset
        axp.set_gpio4_mode(axp192::GpioMode34::NmosOpenDrainOutput).unwrap();
        axp.set_gpio4_output(false).unwrap();
        delay.delay_millis(100u32);
        axp.set_gpio4_output(true).unwrap();
        // power LED
        axp.set_gpio1_mode(axp192::GpioMode12::NmosOpenDrainOutput).unwrap();
        axp.set_gpio1_output(false).unwrap();

        let mut touch = ft6x36::Ft6x36::new(RefCellDevice::new(&i2c_bus), ft6x36::Dimension(320, 240));
        touch.init().unwrap();

        let spi = Spi::new(peripherals.SPI3, 60u32.MHz(), SpiMode::Mode0, &clocks).with_pins(
            Some(io.pins.gpio18), //sclk
            Some(io.pins.gpio23), //sdo
            esp_hal::gpio::NO_PIN,
            esp_hal::gpio::NO_PIN,
        );

        let dc = Output::new(io.pins.gpio15, Level::Low);
        let cs = Output::new(io.pins.gpio5, Level::Low);
        let rst = DummyPin::new_low(); // connected with AXP192

        let spi = embedded_hal_bus::spi::ExclusiveDevice::new_no_delay(spi, cs).unwrap();
        let di = SPIInterface::new(spi, dc);
        let display = mipidsi::Builder::new(mipidsi::models::ILI9342CRgb565, di)
            .reset_pin(rst)
            .display_size(320, 240)
            .invert_colors(mipidsi::options::ColorInversion::Inverted)
            .color_order(mipidsi::options::ColorOrder::Bgr)
            .init(&mut delay)
            .unwrap();

        let size = display.size();
        let size = slint::PhysicalSize::new(size.width, size.height);

        self.window.borrow().as_ref().unwrap().set_size(size);

        let mut buffer_provider = DrawBuffer {
            display,
            buffer: &mut [slint::platform::software_renderer::Rgb565Pixel(0); 320],
        };

        let mut last_touch = None;
        let button = slint::platform::PointerEventButton::Left;

        loop {
            slint::platform::update_timers_and_animations();
            if let Some(window) = self.window.borrow().clone() {
                touch.get_touch_event()
                    .ok()
                    .and_then(|touch_event| {
                        let time = self.duration_since_start();
                        let event = touch_event.p1
                            .map(|p1| {
                                let position = slint::PhysicalPosition::new(
                                    ((p1.x as f32) * size.width as f32 / 319.) as _,
                                    (p1.y as f32 * size.height as f32 / 239.) as _,
                                ).to_logical(window.scale_factor());
                                let _ = last_touch.replace(position);
                                match p1.touch_type {
                                    ft6x36::TouchType::Press => WindowEvent::PointerPressed { position, button },
                                    ft6x36::TouchType::Contact => WindowEvent::PointerMoved { position },
                                    ft6x36::TouchType::Release => todo!(),
                                    ft6x36::TouchType::Invalid => todo!(),
                                }
                            }).or_else(|| {
                                match last_touch {
                                    Some(_) => last_touch.take().map(|position| WindowEvent::PointerReleased {
                                        position,
                                        button,
                                    }),
                                    None => None,
                                }
                            });
                        if !event.is_none() {
                            let e = event.unwrap();
                            let is_pointer_release_event = matches!(e, WindowEvent::PointerReleased { .. });
                            window.dispatch_event(e);
                            // removes hover state on widgets
                            if is_pointer_release_event {
                                window.dispatch_event(WindowEvent::PointerExited);
                            }
                        }
                        touch.process_event(time, touch_event)
                    });

                window.draw_if_needed(|renderer| {
                    renderer.render_by_line(&mut buffer_provider);
                });
                if window.has_active_animations() {
                    continue;
                }
            }
            // TODO
        }
    }

    fn debug_log(&self, arguments: core::fmt::Arguments) {
        esp_println::println!("{}", arguments);
    }
}

struct DrawBuffer<'a, Display> {
    display: Display,
    buffer: &'a mut [slint::platform::software_renderer::Rgb565Pixel],
}

impl<
        DI: display_interface::WriteOnlyDataCommand,
        RST: OutputPin<Error = core::convert::Infallible>,
    > slint::platform::software_renderer::LineBufferProvider
    for &mut DrawBuffer<'_, Display<DI, mipidsi::models::ILI9342CRgb565, RST>>
{
    type TargetPixel = slint::platform::software_renderer::Rgb565Pixel;

    fn process_line(
        &mut self,
        line: usize,
        range: core::ops::Range<usize>,
        render_fn: impl FnOnce(&mut [slint::platform::software_renderer::Rgb565Pixel]),
    ) {
        let buffer = &mut self.buffer[range.clone()];

        render_fn(buffer);

        // We send empty data just to get the device in the right window
        self.display
            .set_pixels(
                range.start as u16,
                line as _,
                range.end as u16,
                line as u16,
                buffer
                    .iter()
                    .map(|x| embedded_graphics_core::pixelcolor::raw::RawU16::new(x.0).into()),
            )
            .unwrap();
    }
}

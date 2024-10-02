// Copyright © SixtyFPS GmbH <info@slint.dev>
// SPDX-License-Identifier: MIT

use alloc::boxed::Box;
use alloc::rc::Rc;
use core::cell::RefCell;
use display_interface_spi::SPIInterface;
use embedded_graphics_core::geometry::OriginDimensions;
use embedded_hal::digital::OutputPin;
use esp_alloc::EspHeap;
use esp_backtrace as _;
use esp_hal::clock::ClockControl;
use esp_hal::delay::Delay;
pub use esp_hal::entry;
use esp_hal::gpio::{Io, Level, Output};
use esp_hal::rtc_cntl::Rtc;
use esp_hal::spi::{master::Spi, SpiMode};
use esp_hal::system::SystemControl;
use esp_hal::timer::{systimer::SystemTimer, timg::TimerGroup};
use esp_hal::{i2c::I2C, peripherals::Peripherals, prelude::*};
use mipidsi::{options::Orientation, Display};
use slint::platform::WindowEvent;
use esp_println::println;

#[global_allocator]
static ALLOCATOR: EspHeap = EspHeap::empty();

pub fn init() {
    const HEAP_SIZE: usize = 250 * 1024;
    static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];
    unsafe { ALLOCATOR.init(core::ptr::addr_of_mut!(HEAP) as *mut u8, HEAP_SIZE) }
    slint::platform::set_platform(Box::new(EspBackend::default()))
        .expect("backend already initialized");
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
            SystemTimer::now() / (SystemTimer::TICKS_PER_SECOND / 1000),
        )
    }

    fn run_event_loop(&self) -> Result<(), slint::PlatformError> {
        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::max(system.clock_control).freeze();

        let mut rtc = Rtc::new(peripherals.LPWR, None);
        rtc.rwdt.disable();
        let mut timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks, None);
        timer_group0.wdt.disable();
        let mut timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks, None);
        timer_group1.wdt.disable();

        let mut delay = Delay::new(&clocks);
        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

        let i2c =
            I2C::new(peripherals.I2C0, io.pins.gpio2, io.pins.gpio1, 400u32.kHz(), &clocks, None);

        let mut touch = ft6x36::Ft6x36::new(i2c, ft6x36::Dimension(480, 320));
        touch.init().unwrap();
        match touch.get_info() {
            Some(info) => println!("Touch screen info: {info:?}"),
            None => println!("No Info from Touch screen"),
        }

        let sclk = io.pins.gpio12;
        let mosi = io.pins.gpio13;

        let spi = Spi::new(peripherals.SPI3, 60u32.MHz(), SpiMode::Mode0, &clocks).with_pins(
            Some(sclk),
            Some(mosi),
            esp_hal::gpio::NO_PIN,
            esp_hal::gpio::NO_PIN,
        );

        let dc = Output::new(io.pins.gpio42, Level::Low);
        let cs = Output::new(io.pins.gpio3, Level::Low);
        // let rst = Output::new(io.pins.gpio40, Level::Low);

        let spi = embedded_hal_bus::spi::ExclusiveDevice::new_no_delay(spi, cs).unwrap();

        let di = SPIInterface::new(spi, dc);
        let display = mipidsi::Builder::new(mipidsi::models::ILI9486Rgb666, di)
            // .reset_pin(rst)
            .orientation(Orientation::new().rotate(mipidsi::options::Rotation::Deg90).flip_horizontal())
            .color_order(mipidsi::options::ColorOrder::Bgr)
            .init(&mut delay)
            .unwrap();

        let mut backlight = Output::new(io.pins.gpio46, Level::High);
        backlight.set_high();

        let size = display.size();
        let size = slint::PhysicalSize::new(size.width, size.height);
        println!("Display size: {:?}", size);

        self.window.borrow().as_ref().unwrap().set_size(size);

        let mut buffer_provider = DrawBuffer {
            display,
            buffer: &mut [slint::Rgb8Pixel::new(0, 0, 0); 480],
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
                                    (479. - ((p1.y as f32) * size.width as f32 / 479.)) as _,
                                    ((p1.x as f32) * size.height as f32 / 319.) as _,
                                ).to_logical(window.scale_factor());
                                let _ = last_touch.replace(position);
                                match p1.touch_type {
                                    ft6x36::TouchType::Press => WindowEvent::PointerPressed { position, button },
                                    ft6x36::TouchType::Contact => WindowEvent::PointerMoved { position },
                                    ft6x36::TouchType::Release => WindowEvent::PointerReleased { position, button },
                                    ft6x36::TouchType::Invalid => WindowEvent::PointerMoved { position },
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
                            println!("Touch event: {:?}", e);
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
    buffer: &'a mut [slint::Rgb8Pixel],
}

impl<
        DI: display_interface::WriteOnlyDataCommand,
        RST: OutputPin<Error = core::convert::Infallible>,
    > slint::platform::software_renderer::LineBufferProvider
    for &mut DrawBuffer<'_, Display<DI, mipidsi::models::ILI9486Rgb666, RST>>
{
    type TargetPixel = slint::Rgb8Pixel;

    fn process_line(
        &mut self,
        line: usize,
        range: core::ops::Range<usize>,
        render_fn: impl FnOnce(&mut [slint::Rgb8Pixel]),
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
                    .map(|x| embedded_graphics_core::pixelcolor::Rgb888::new(x.r, x.g, x.b).into())
            )
            .unwrap();
    }
}

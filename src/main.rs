#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::{cell::RefCell, convert::Infallible};

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::{
    adc, gpio, pwm,
    uart::{self, Blocking, UartTx},
};
use embassy_rp::{bind_interrupts, i2c};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};

use embassy_time::Timer;
use embedded_ccs811::{self as ccs811, nb, prelude::*};
use embedded_hal as hal;

use {defmt_rtt as _, panic_probe as _};

mod bmp280;
mod server;

type Uart = UartTx<'static, embassy_rp::peripherals::UART0, Blocking>;

#[derive(Copy, Clone, Debug)]
pub struct DataPoint {
    temp: f32,
    humidity: f32,
    pressure: f32,
    co: u16,
    co2: u16,
    tvoc: u16,
}

mod alarm_thresholds {
    pub const HUMIDITY: f32 = 70.0;
    pub const CO: u16 = 200;
    pub const CO2: u16 = 1000;
    pub const TVOC: u16 = 400;
}

bind_interrupts!(struct Irqs {
    ADC_IRQ_FIFO => adc::InterruptHandler;
});

pub static DATA_POINT: Mutex<ThreadModeRawMutex, RefCell<Option<DataPoint>>> =
    Mutex::new(RefCell::new(None));

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    info!("Starting webserver");
    spawner.must_spawn(server::start(
        p.PIN_23, p.PIN_24, p.PIN_25, p.PIN_29, p.PIO0, p.DMA_CH0,
    ));

    info!("Setup analog in and pwm current control for CO sensor");
    let mut adc = adc::Adc::new(p.ADC, Irqs, Default::default());
    let mut co_chan = adc::Channel::new_pin(p.PIN_26, embassy_rp::gpio::Pull::None);
    let mut co_conf = pwm::Config::default();
    co_conf.top = 100;
    co_conf.compare_b = 100;
    let mut _co_pwm = pwm::Pwm::new_output_b(p.PWM_CH0, p.PIN_17, co_conf.clone());

    info!("Setup i2c for humidity, temp and pressure sensor");
    let mut temp_i2c = i2c::I2c::new_blocking(p.I2C0, p.PIN_5, p.PIN_4, i2c::Config::default());
    let calib = bmp280::get_calib(&mut temp_i2c).unwrap();
    bmp280::init(&mut temp_i2c).unwrap();

    info!("Setup i2c for CO2 and VOC sensor");
    let co2_i2c = i2c::I2c::new_blocking(p.I2C1, p.PIN_15, p.PIN_14, i2c::Config::default());
    let ccs811 = ccs811::Ccs811Awake::new(co2_i2c, ccs811::SlaveAddr::default());
    let mut ccs811 = ccs811.start_application().ok().unwrap();
    ccs811
        .set_mode(ccs811::MeasurementMode::ConstantPower1s)
        .unwrap();

    info!("Setup alarm LEDs");
    let mut co_alarm = gpio::Output::new(p.PIN_9, gpio::Level::Low);
    let mut co2_alarm = gpio::Output::new(p.PIN_7, gpio::Level::Low);
    let mut hum_alarm = gpio::Output::new(p.PIN_6, gpio::Level::Low);
    let mut tvoc_alarm = gpio::Output::new(p.PIN_8, gpio::Level::Low);

    // wait some time to have the display booted up
    Timer::after_secs(1).await;

    info!("Setup UART display comm");
    let mut uart_cfg = uart::Config::default();
    uart_cfg.baudrate = 9600;
    let mut uart_tx = UartTx::new(p.UART0, p.PIN_16, p.DMA_CH1, uart_cfg);

    let mut fmt_buf = [0u8; 64];

    loop {
        uart_tx.blocking_write(&[0xfe, 0x0c]).unwrap();
        // reset display
        let co = adc.read(&mut co_chan).await.unwrap();

        let bmp280_res = bmp280::read(&mut temp_i2c, &calib).unwrap();

        ccs811
            .set_environment(bmp280_res.humidity, bmp280_res.temperature)
            .unwrap();

        let co2_tvoc = nb::block!(ccs811.data()).unwrap();

        info!(
            "{:?}, CO: {}, CO2: {}, TVAC: {}",
            defmt::Debug2Format(&bmp280_res),
            co,
            co2_tvoc.eco2,
            co2_tvoc.etvoc
        );

        let dp = DataPoint {
            temp: bmp280_res.temperature,
            humidity: bmp280_res.humidity,
            pressure: bmp280_res.pressure,
            co,
            co2: co2_tvoc.eco2,
            tvoc: co2_tvoc.etvoc,
        };

        check_alarms(
            &dp,
            &mut co_alarm,
            &mut co2_alarm,
            &mut hum_alarm,
            &mut tvoc_alarm,
        )
        .unwrap();

        {
            let lock = DATA_POINT.lock().await;
            lock.replace(Some(dp));
        }

        let ts = format_no_std::show(
            &mut fmt_buf,
            format_args!("Temp: {:.2}C", bmp280_res.temperature),
        )
        .unwrap();
        write_x_y(&mut uart_tx, 5, 0, ts.as_bytes()).unwrap();

        let ts = format_no_std::show(
            &mut fmt_buf,
            format_args!("Humidity: {:.2}%RH", bmp280_res.humidity),
        )
        .unwrap();
        write_x_y(&mut uart_tx, 1, 1, ts.as_bytes()).unwrap();

        let ts = format_no_std::show(
            &mut fmt_buf,
            format_args!("Pressure: {:.1}hPa", bmp280_res.pressure),
        )
        .unwrap();
        write_x_y(&mut uart_tx, 1, 2, ts.as_bytes()).unwrap();
        Timer::after_secs(3).await;
    }
}

fn write_x_y(uart: &mut Uart, x: u8, y: u8, txt: &[u8]) -> Result<(), uart::Error> {
    const LINE_OFFSETS: [u8; 4] = [0, 64, 20, 84];
    uart.blocking_write(&[0xfe, 0x80 + LINE_OFFSETS[y as usize] + x])?;
    uart.blocking_write(txt)?;
    Ok(())
}

fn check_alarms<E: hal::digital::Error>(
    data: &DataPoint,
    co: &mut dyn hal::digital::OutputPin<Error = E>,
    co2: &mut dyn hal::digital::OutputPin<Error = E>,
    hum: &mut dyn hal::digital::OutputPin<Error = E>,
    tvoc: &mut dyn hal::digital::OutputPin<Error = E>,
) -> Result<bool, Infallible> {
    let mut any_alarm = false;

    if data.co > alarm_thresholds::CO {
        co.set_high().unwrap();
        any_alarm = true;
    } else {
        co.set_low().unwrap();
    }
    if data.co > alarm_thresholds::CO2 {
        co2.set_high().unwrap();
        any_alarm = true;
    } else {
        co2.set_low().unwrap();
    }
    if data.humidity > alarm_thresholds::HUMIDITY {
        hum.set_high().unwrap();
        any_alarm = true;
    } else {
        hum.set_low().unwrap();
    }
    if data.tvoc > alarm_thresholds::TVOC {
        tvoc.set_high().unwrap();
        any_alarm = true;
    } else {
        tvoc.set_low().unwrap();
    }

    Ok(any_alarm)
}

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use bmp2080::Calib;
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::i2c;
use embassy_rp::uart::{self, Blocking, UartTx};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use embassy_sync::pubsub::PubSubChannel;
use embassy_time::Timer;
use embedded_hal::i2c::I2c;

use {defmt_rtt as _, panic_probe as _};

mod server;

mod bmp2080 {
    pub const ADDR: u8 = 0x76;

    pub const REG_CONFIG: u8 = 0xf5;
    pub const REG_CTRL_MEAS: u8 = 0xf4;
    pub const REG_CTRL_HUM: u8 = 0xf2;
    pub const REG_DATA_START: u8 = 0xf7;

    pub const P_T_CALIB_VALUES: usize = 26;
    pub const P_T_CALIB_DATA_START: u8 = 0x88;
    pub const H_CALIB_VALUES: usize = 7;
    pub const H_CALIB_DATA_START: u8 = 0xe1;

    pub struct Calib {
        pub t1: u32,
        pub t2: i32,
        pub t3: i32,
        pub p1: u64,
        pub p2: i64,
        pub p3: i64,
        pub p4: i64,
        pub p5: i64,
        pub p6: i64,
        pub p7: i64,
        pub p8: i64,
        pub p9: i64,
        pub h1: u32,
        pub h2: i32,
        pub h3: u32,
        pub h4: i32,
        pub h5: i32,
        pub h6: i32,
    }
}

type Uart = UartTx<'static, embassy_rp::peripherals::UART0, Blocking>;

#[derive(Copy, Clone, Debug)]
struct DataPoint {
    temp: f32,
    humidity: f32,
    pressure: f32,
}

pub static CHANNEL: PubSubChannel<CriticalSectionRawMutex, DataPoint, 1, 1, 1> =
    PubSubChannel::<CriticalSectionRawMutex, DataPoint, 1, 1, 1>::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    spawner
        .spawn(server::start(
            p.PIN_23, p.PIN_24, p.PIN_25, p.PIN_29, p.PIO0, p.DMA_CH0,
        ))
        .unwrap();

    let sda = p.PIN_4;
    let scl = p.PIN_5;

    info!("Setup i2c");

    let mut i2c = i2c::I2c::new_blocking(p.I2C0, scl, sda, i2c::Config::default());
    let calib = bmp280_get_calib(&mut i2c).unwrap();
    bmp280_init(&mut i2c).unwrap();

    let mut uart_cfg = uart::Config::default();

    // wait some time to have the display booted up
    Timer::after_secs(1).await;

    uart_cfg.baudrate = 9600;
    let mut uart_tx = UartTx::new(p.UART0, p.PIN_16, p.DMA_CH1, uart_cfg);

    let mut fmt_buf = [0u8; 64];

    let publisher = CHANNEL.publisher().unwrap();

    loop {
        uart_tx.blocking_write(&[0xfe, 0x0c]).unwrap();
        // reset display
        let (p, t, h) = bmp280_read_raw(&mut i2c).unwrap();
        let fine = bmp280_convert(t as i32, &calib);
        let temp = bmp280_convert_temp(fine);
        let press = bmp280_convert_pressure(p, fine, &calib);
        let hum = bmp280_convert_humidity(h, fine, &calib);

        info!("Pressure: {}, Temp: {}, Humidity: {}", press, temp, hum);

        publisher.publish_immediate(DataPoint {
            temp,
            humidity: hum,
            pressure: press,
        });

        let ts = format_no_std::show(&mut fmt_buf, format_args!("Temp: {:.2}C", temp)).unwrap();
        write_x_y(&mut uart_tx, 5, 0, ts.as_bytes()).unwrap();

        let ts =
            format_no_std::show(&mut fmt_buf, format_args!("Humidity: {:.2}%RH", hum)).unwrap();
        write_x_y(&mut uart_tx, 1, 1, ts.as_bytes()).unwrap();

        let ts =
            format_no_std::show(&mut fmt_buf, format_args!("Pressure: {:.1}hPa", press)).unwrap();
        write_x_y(&mut uart_tx, 1, 2, ts.as_bytes()).unwrap();
        Timer::after_secs(3).await;
    }
}

fn bmp280_get_calib(i2c: &mut dyn I2c<Error = i2c::Error>) -> Result<bmp2080::Calib, i2c::Error> {
    use bmp2080::*;
    let mut p_t_buf = [0; P_T_CALIB_VALUES];
    i2c.write_read(ADDR, &[P_T_CALIB_DATA_START], &mut p_t_buf)?;
    let mut h_buf = [0; H_CALIB_VALUES];
    i2c.write_read(ADDR, &[H_CALIB_DATA_START], &mut h_buf)?;
    let calib = Calib {
        t1: (p_t_buf[1] as u32) << 8 | p_t_buf[0] as u32,
        t2: (p_t_buf[3] as i32) << 8 | p_t_buf[2] as i32,
        t3: (p_t_buf[5] as i32) << 8 | p_t_buf[4] as i32,
        p1: (p_t_buf[7] as u64) << 8 | p_t_buf[6] as u64,
        p2: (p_t_buf[9] as i64) << 8 | p_t_buf[8] as i64,
        p3: (p_t_buf[11] as i64) << 8 | p_t_buf[10] as i64,
        p4: (p_t_buf[13] as i64) << 8 | p_t_buf[12] as i64,
        p5: (p_t_buf[15] as i64) << 8 | p_t_buf[14] as i64,
        p6: (p_t_buf[17] as i64) << 8 | p_t_buf[16] as i64,
        p7: (p_t_buf[19] as i64) << 8 | p_t_buf[18] as i64,
        p8: (p_t_buf[21] as i64) << 8 | p_t_buf[20] as i64,
        p9: (p_t_buf[23] as i64) << 8 | p_t_buf[22] as i64,
        h1: (p_t_buf[25] as u32),
        h2: (h_buf[1] as i32) << 8 | p_t_buf[0] as i32,
        h3: (h_buf[2] as u32),
        h4: ((h_buf[3] as i32) << 4) | (h_buf[4] as i32 & 0x0f),
        h5: ((h_buf[5] as i32) << 4) | ((h_buf[4] as i32 >> 4) | 0x0f),
        h6: h_buf[6] as i32,
    };
    Ok(calib)
}

fn bmp280_convert(temp: i32, calib: &Calib) -> i32 {
    let var1 = temp as f32 / 16384.0 - calib.t1 as f32 / 1024.0;
    let var1 = var1 * calib.t2 as f32;
    let var2 = temp as f32 / 131072.0 - calib.t1 as f32 / 8192.0;
    let var2 = var2 * var2 * calib.t3 as f32;

    (var1 + var2) as i32
}

fn bmp280_convert_temp(fine: i32) -> f32 {
    fine as f32 / 5120.0
}

fn bmp280_convert_pressure(press: u32, fine: i32, calib: &Calib) -> f32 {
    let var1 = fine as f32 / 2.0 - 64000.0;
    let var2 = var1 * var1 * calib.p6 as f32 / 32768.0;
    let var2 = var2 + var1 * calib.p5 as f32 * 2.0;
    let var2 = var2 / 4.0 + calib.p4 as f32 * 65536.0;
    let var3 = calib.p3 as f32 * var1 * var1 / 524288.0;
    let var1 = (var3 + calib.p2 as f32 * var1) / 524288.0;
    let var1 = (1.0 + var1 / 32768.0) * calib.p1 as f32;
    let pressure = 1048576.0 - press as f32;
    let pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
    let var1 = calib.p9 as f32 * pressure * pressure / 2147483648.0;
    let var2 = pressure * calib.p8 as f32 / 32768.0;
    let pressure = pressure + (var1 + var2 + calib.p7 as f32) / 16.0;
    pressure / 100.0
}

fn bmp280_convert_humidity(hum: u32, fine: i32, calib: &Calib) -> f32 {
    let var1 = fine as f32 - 76800.0;
    let var2 = calib.h4 as f32 * 64.0 + (calib.h5 as f32 / 16384.0) * var1;
    let var3 = hum as f32 - var2;
    let var4 = calib.h2 as f32 / 65536.0;
    let var5 = 1.0 + (calib.h3 as f32 / 67108864.0) * var1;
    let var6 = 1.0 + (calib.h6 as f32 / 67108864.0) * var1 * var5;
    let var6 = var3 * var4 * var5 * var6;

    let humidity = var6 * (1.0 - calib.h1 as f32 * var6 / 524288.0);
    humidity.max(0.0).min(100.0)
}

fn write_x_y(uart: &mut Uart, x: u8, y: u8, txt: &[u8]) -> Result<(), uart::Error> {
    const LINE_OFFSETS: [u8; 4] = [0, 64, 20, 84];
    uart.blocking_write(&[0xfe, 0x80 + LINE_OFFSETS[y as usize] + x])?;
    uart.blocking_write(txt)?;
    Ok(())
}

fn bmp280_init(i2c: &mut dyn I2c<Error = i2c::Error>) -> Result<(), i2c::Error> {
    use bmp2080::*;

    // 1000ms sampling time, filter off
    i2c.write(ADDR, &[REG_CONFIG, (0x05 << 5)])?;
    // osrs_h x1 NOTE: Must be wrtten before writing to REG_CTRL_MEAS
    i2c.write(ADDR, &[REG_CTRL_HUM, 0x01])?;
    // osrs_t x1, osrs_p x1, normal mode operation
    i2c.write(ADDR, &[REG_CTRL_MEAS, (0x01 << 5) | (0x01 << 2) | 0x03])?;

    Ok(())
}

fn bmp280_read_raw(i2c: &mut dyn I2c<Error = i2c::Error>) -> Result<(u32, u32, u32), i2c::Error> {
    use bmp2080::*;

    let mut buf = [0; 8];
    i2c.write_read(ADDR, &[REG_DATA_START], &mut buf)?;

    let pressure = ((buf[0] as u32) << 12) | ((buf[1] as u32) << 4) | ((buf[2] as u32) >> 4);
    let temp = ((buf[3] as u32) << 12) | ((buf[4] as u32) << 4) | ((buf[5] as u32) >> 4);
    let hum = ((buf[6] as u32) << 8) | (buf[7] as u32);

    Ok((pressure, temp, hum))
}

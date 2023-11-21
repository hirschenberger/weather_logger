use embedded_hal::i2c::{self, I2c};

const ADDR: u8 = 0x76;

const REG_CONFIG: u8 = 0xf5;
const REG_CTRL_MEAS: u8 = 0xf4;
const REG_CTRL_HUM: u8 = 0xf2;
const REG_DATA_START: u8 = 0xf7;

const P_T_CALIB_VALUES: usize = 26;
const P_T_CALIB_DATA_START: u8 = 0x88;
const H_CALIB_VALUES: usize = 7;
const H_CALIB_DATA_START: u8 = 0xe1;

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

#[derive(Clone, Copy, Debug)]
pub struct Bmp280Read {
    pub pressure: f32,
    pub humidity: f32,
    pub temperature: f32,
}

pub fn init<E: i2c::Error>(i2c: &mut dyn I2c<Error = E>) -> Result<(), E> {
    // 1000ms sampling time, filter off
    i2c.write(ADDR, &[REG_CONFIG, (0x05 << 5)])?;
    // osrs_h x1 NOTE: Must be wrtten before writing to REG_CTRL_MEAS
    i2c.write(ADDR, &[REG_CTRL_HUM, 0x01])?;
    // osrs_t x1, osrs_p x1, normal mode operation
    i2c.write(ADDR, &[REG_CTRL_MEAS, (0x01 << 5) | (0x01 << 2) | 0x03])?;

    Ok(())
}

pub fn get_calib<E: i2c::Error>(i2c: &mut dyn I2c<Error = E>) -> Result<Calib, E> {
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

pub fn read<E: i2c::Error>(i2c: &mut dyn I2c<Error = E>, calib: &Calib) -> Result<Bmp280Read, E> {
    let (p, t, h) = read_raw(i2c).unwrap();
    let fine = convert(t as i32, calib);
    let temperature = convert_temp(fine);
    let pressure = convert_pressure(p, fine, calib);
    let humidity = convert_humidity(h, fine, calib);
    Ok(Bmp280Read {
        pressure,
        humidity,
        temperature,
    })
}

fn read_raw<E: i2c::Error>(i2c: &mut dyn I2c<Error = E>) -> Result<(u32, u32, u32), E> {
    let mut buf = [0; 8];
    i2c.write_read(ADDR, &[REG_DATA_START], &mut buf)?;

    let pressure = ((buf[0] as u32) << 12) | ((buf[1] as u32) << 4) | ((buf[2] as u32) >> 4);
    let temp = ((buf[3] as u32) << 12) | ((buf[4] as u32) << 4) | ((buf[5] as u32) >> 4);
    let hum = ((buf[6] as u32) << 8) | (buf[7] as u32);

    Ok((pressure, temp, hum))
}

fn convert(temp: i32, calib: &Calib) -> i32 {
    let var1 = temp as f32 / 16384.0 - calib.t1 as f32 / 1024.0;
    let var1 = var1 * calib.t2 as f32;
    let var2 = temp as f32 / 131072.0 - calib.t1 as f32 / 8192.0;
    let var2 = var2 * var2 * calib.t3 as f32;

    (var1 + var2) as i32
}
fn convert_temp(fine: i32) -> f32 {
    fine as f32 / 5120.0
}

fn convert_pressure(press: u32, fine: i32, calib: &Calib) -> f32 {
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

fn convert_humidity(hum: u32, fine: i32, calib: &Calib) -> f32 {
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

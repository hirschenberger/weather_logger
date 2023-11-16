use cyw43_pio::PioSpi;
use defmt::*;
use embassy_executor::Spawner;
use embassy_net::{tcp::TcpSocket, Config, Stack, StackResources};
use embassy_rp::{
    bind_interrupts,
    gpio::{Level, Output},
    peripherals::{DMA_CH0, PIN_23, PIN_24, PIN_25, PIO0},
    pio::{InterruptHandler, Pio},
};
use embassy_time::Duration;
use embedded_io_async::Write;
use static_cell::make_static;

use {defmt_rtt as _, panic_probe as _};

const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PASS: &str = env!("WIFI_PASS");

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

#[embassy_executor::task]
async fn wifi_task(
    runner: cyw43::Runner<
        'static,
        Output<'static, PIN_23>,
        PioSpi<'static, PIN_25, PIO0, 0, DMA_CH0>,
    >,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<cyw43::NetDriver<'static>>) -> ! {
    stack.run().await
}

#[embassy_executor::task]
pub async fn start(
    p23: PIN_23,
    p24: PIN_24,
    p25: PIN_25,
    p29: embassy_rp::peripherals::PIN_29,
    pio0: PIO0,
    dma_ch0: DMA_CH0,
) {
    // let fw = include_bytes!("../assets/cyw43-firmware/43439A0.bin");
    // let clm = include_bytes!("../assets/cyw43-firmware/43439A0_clm.bin");
    // To make flashing faster for development, you may want to flash the firmwares independently
    // at hardcoded addresses, instead of baking them into the program with `include_bytes!`:
    //     probe-rs download 43439A0.bin --format bin --chip RP2040 --base-address 0x10100000
    //     probe-rs download 43439A0_clm.bin --format bin --chip RP2040 --base-address 0x10140000
    let fw = unsafe { core::slice::from_raw_parts(0x10100000 as *const u8, 230321) };
    let clm = unsafe { core::slice::from_raw_parts(0x10140000 as *const u8, 4752) };

    let pwr = Output::new(p23, Level::Low);
    let cs = Output::new(p25, Level::High);
    let mut pio = Pio::new(pio0, Irqs);
    let spi = PioSpi::new(&mut pio.common, pio.sm0, pio.irq0, cs, p24, p29, dma_ch0);

    'reconnect: loop {
        let state = make_static!(cyw43::State::new());
        let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
        Spawner::for_current_executor()
            .await
            .must_spawn(wifi_task(runner));

        control.init(clm).await;
        control
            .set_power_management(cyw43::PowerManagementMode::Performance)
            .await;

        let config = Config::dhcpv4(Default::default());
        let seed = 0x635424974f;

        let stack = &*make_static!(Stack::new(
            net_device,
            config,
            make_static!(StackResources::<4>::new()),
            seed
        ));

        Spawner::for_current_executor()
            .await
            .must_spawn(net_task(stack));

        loop {
            match control.join_wpa2(WIFI_SSID, WIFI_PASS).await {
                Ok(_) => {
                    info!("Connected to {}", WIFI_SSID);
                    break;
                }
                Err(err) => error!("Can't connect: {}", err.status),
            }
        }

        info!("Waiting fpr DHCP");
        stack.wait_config_up().await;
        info!("DHCP is up");

        let mut rx_buffer = [0; 1];
        let mut tx_buffer = [0; 4096];
        let mut prom_buf = [0; 4096];
        let mut http_buf = [0; 4096];

        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        loop {
            info!("Socket state: {:?}", socket.state());
            control.gpio_set(0, false).await;
            info!("Listening on TCP:1234");
            if let Err(e) = socket.accept(1234).await {
                warn!("Accept error: {:?}", e);
                continue;
            }
            control.gpio_set(0, true).await;

            info!("Received connection from {:?}", socket.remote_endpoint());

            let mut subscriber = crate::CHANNEL.subscriber().unwrap();

            let data = subscriber.next_message_pure().await;
            let prometheus_ex = format_no_std::show(
                &mut prom_buf,
                format_args!(
                    "humidity {:.2}\ntemperature {:.2}\npressure {:.2}\nco {}",
                    data.humidity, data.temp, data.pressure, data.co
                ),
            )
            .unwrap();

            let http = format_no_std::show(
                &mut http_buf,
                format_args!(
                    "HTTP/1.1 200 OK\nContent-Length: {}\nContent-Type: text/plain\n\n{}",
                    prometheus_ex.len(),
                    prometheus_ex
                ),
            )
            .unwrap();

            if let Err(e) = socket.write_all(http.as_bytes()).await {
                warn!("Send error: {:?}", e);
                break 'reconnect;
            }
            socket.flush().await.unwrap();
            socket.abort();
        }
    }
}

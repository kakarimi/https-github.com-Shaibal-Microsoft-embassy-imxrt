#![no_std]
#![no_main]

use core::arch::asm;

use defmt::{assert, info};
use embassy_executor::Spawner;
use embassy_imxrt::flexspi_nor_storage_bus::{
    AhbConfig, FlexSpiBusWidth, FlexSpiFlashPort, FlexSpiFlashPortDeviceInstance, FlexspiAhbBufferConfig,
    FlexspiAhbWriteWaitUnit, FlexspiConfig, FlexspiCsIntervalCycleUnit, FlexspiDeviceConfig, FlexspiNorStorageBus,
    FlexspiReadSampleClock,
};
use embassy_time::Timer;
use embedded_storage::nor_flash::{
    ErrorType, NorFlash as BlockingNorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash as BlockingReadNorFlash,
};
use embedded_storage_async::nor_flash::{NorFlash as AsyncNorFlash, ReadNorFlash as AsyncReadNorFlash};
use storage_bus::storage_bus::{
    BlockingNorStorageBusDriver, DummyCycles, NorStorageBusWidth, NorStorageCmd, NorStorageCmdMode, NorStorageCmdType,
};
use {defmt_rtt as _, panic_probe as _};

static ADDR: u32 = 0x1000;

macro_rules! align {
    ($val:expr, $align:expr) => {
        (($val + $align - 1) / $align) * $align
    };
}

mod sealed {
    /// simply seal a trait
    pub trait Sealed {}
}

impl<T> sealed::Sealed for T {}

/// Driver mode.
#[allow(private_bounds)]
pub trait Mode: sealed::Sealed {}

/// Blocking mode.
pub struct Blocking;
impl Mode for Blocking {}

/// Async mode.
pub struct Async;
impl Mode for Async {}

struct MacronixDeviceDriver<T: BlockingNorStorageBusDriver, M: Mode> {
    // Bus driver dependency
    storagebusdriver: T,
    capacity: usize,
    _phantom: core::marker::PhantomData<M>,
}

/// Nor flash error object
#[derive(Debug)]
pub struct FlashStorageErrorOther;
impl<T: BlockingNorStorageBusDriver, M: Mode> ErrorType for MacronixDeviceDriver<T, M> {
    type Error = FlashStorageErrorOther;
}

impl NorFlashError for FlashStorageErrorOther {
    fn kind(&self) -> NorFlashErrorKind {
        NorFlashErrorKind::Other
    }
}

impl<T: BlockingNorStorageBusDriver> BlockingReadNorFlash for MacronixDeviceDriver<T, Blocking> {
    const READ_SIZE: usize = 256;
    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        let mut buffer = [0_u8; Self::READ_SIZE];
        let mut read_start_ptr = 0;
        let mut read_end_ptr = bytes.len();
        if bytes.len() <= Self::READ_SIZE {
            info!("Reading data in single shot");
            let read_cmd = NorStorageCmd {
                cmd_lb: 0xEE,
                cmd_ub: Some(0x11),
                addr: Some(offset),
                addr_width: Some(4),
                bus_width: NorStorageBusWidth::Octal, // 3 - Octal
                mode: NorStorageCmdMode::DDR,
                dummy: DummyCycles::Clocks(0x3),
                cmdtype: Some(NorStorageCmdType::Read),
                data_bytes: Some(bytes.len() as u32),
            };

            let _ = self.storagebusdriver.send_command(read_cmd, Some(&mut buffer), None);
        } else {
            loop {
                if bytes.len() - read_start_ptr < Self::READ_SIZE {
                    break;
                }
                // Read data from the storage device
                let read_cmd = NorStorageCmd {
                    cmd_lb: 0xEE,
                    cmd_ub: Some(0x11),
                    addr: Some(offset),
                    addr_width: Some(4),
                    bus_width: NorStorageBusWidth::Octal,
                    mode: NorStorageCmdMode::DDR,
                    dummy: DummyCycles::Clocks(20),
                    cmdtype: Some(NorStorageCmdType::Read),
                    data_bytes: Some(Self::READ_SIZE as u32),
                };

                let _ = self.storagebusdriver.send_command(read_cmd, Some(&mut buffer), None);

                for i in 0..Self::READ_SIZE {
                    bytes[read_start_ptr] = buffer[i];
                    read_start_ptr += 1;
                }
            }
            if bytes.len() - read_start_ptr > 0 {
                let read_cmd = NorStorageCmd {
                    cmd_lb: 0xEE,
                    cmd_ub: Some(0x11),
                    addr: Some(offset),
                    addr_width: Some(4),
                    bus_width: NorStorageBusWidth::Octal,
                    mode: NorStorageCmdMode::DDR,
                    dummy: DummyCycles::Clocks(20),
                    cmdtype: Some(NorStorageCmdType::Read),
                    data_bytes: Some((bytes.len() - read_start_ptr) as u32),
                };

                let _ = self.storagebusdriver.send_command(read_cmd, Some(&mut buffer), None);

                for i in 0..(bytes.len() - read_start_ptr) {
                    bytes[read_start_ptr] = buffer[i];
                    read_start_ptr += 1;
                }
            }
        }
        Ok(())
    }
    fn capacity(&self) -> usize {
        self.capacity
    }
}

impl<T: BlockingNorStorageBusDriver> BlockingNorFlash for MacronixDeviceDriver<T, Blocking> {
    const WRITE_SIZE: usize = 256;
    const ERASE_SIZE: usize = 4096;

    fn erase(&mut self, mut from: u32, to: u32) -> Result<(), Self::Error> {
        // subtracting 1 as align will give next sector start address
        let mut sector_start_addr = 0;
        let mut sector_end_addr = 0;
        if from % Self::ERASE_SIZE as u32 != 0 {
            sector_start_addr = align!(from, Self::ERASE_SIZE as u32) - 1;
        }
        if to % Self::ERASE_SIZE as u32 != 0 {
            sector_end_addr = align!(to, Self::ERASE_SIZE as u32) - 1;
        }
        loop {
            if sector_start_addr == sector_end_addr {
                break;
            }
            let _ = self.storagebusdriver.send_command(
                NorStorageCmd {
                    cmd_lb: 0x21,
                    cmd_ub: Some(0xDE),
                    addr: Some(sector_start_addr),
                    addr_width: Some(4),
                    bus_width: NorStorageBusWidth::Octal,
                    mode: NorStorageCmdMode::DDR,
                    dummy: DummyCycles::Clocks(0),
                    cmdtype: None,
                    data_bytes: None,
                },
                None,
                None,
            );
            sector_start_addr += Self::ERASE_SIZE as u32;
        }
        Ok(())
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        let bus_ref = &mut self.storagebusdriver;
        let mut status = [0_u8; 3];

        // Enable Write
        let write_enable_cmd = NorStorageCmd {
            cmd_lb: 0x06,
            cmd_ub: Some(0xF9),
            addr: None,
            addr_width: None,
            bus_width: NorStorageBusWidth::Octal,
            mode: NorStorageCmdMode::DDR,
            dummy: DummyCycles::Clocks(0),
            cmdtype: None,
            data_bytes: None,
        };
        let _ = bus_ref.send_command(write_enable_cmd, None, None);

        // Check write enable status
        let read_status_cmd = NorStorageCmd {
            cmd_lb: 0x05,
            cmd_ub: Some(0xFA),
            addr: None,
            addr_width: Some(4),
            bus_width: NorStorageBusWidth::Octal,
            mode: NorStorageCmdMode::DDR,
            dummy: DummyCycles::Clocks(0x14),
            cmdtype: Some(NorStorageCmdType::Read),
            data_bytes: Some(1),
        };
        let _ = bus_ref.send_command(read_status_cmd, Some(&mut status), None);

        info!("Status: {:?}", status);

        // Page Program
        let mut write_start_ptr = 0;
        let mut write_end_ptr = Self::WRITE_SIZE;
        if bytes.len() <= Self::WRITE_SIZE {
        } else {
            loop {
                if bytes.len() - write_start_ptr < Self::WRITE_SIZE {
                    break;
                }
                let write_cmd = NorStorageCmd {
                    cmd_lb: 0x12,
                    cmd_ub: Some(0xED),
                    addr: Some(offset),
                    addr_width: Some(4),
                    bus_width: NorStorageBusWidth::Octal,
                    mode: NorStorageCmdMode::DDR,
                    dummy: DummyCycles::Clocks(0),
                    cmdtype: None,
                    data_bytes: Some(Self::WRITE_SIZE as u32),
                };
                let _ = bus_ref.send_command(write_cmd, None, Some(&bytes[write_start_ptr..write_end_ptr]));
                write_start_ptr += Self::WRITE_SIZE;
                write_end_ptr += Self::WRITE_SIZE;
            }
            if bytes.len() - write_start_ptr > 0 {
                let write_cmd = NorStorageCmd {
                    cmd_lb: 0x12,
                    cmd_ub: Some(0xED),
                    addr: Some(offset),
                    addr_width: Some(4),
                    bus_width: NorStorageBusWidth::Octal,
                    mode: NorStorageCmdMode::DDR,
                    dummy: DummyCycles::Clocks(0),
                    cmdtype: None,
                    data_bytes: Some((bytes.len() - write_start_ptr) as u32),
                };
                let _ = bus_ref.send_command(write_cmd, None, Some(&bytes[write_start_ptr..bytes.len()]));
            }
        }

        Ok(())
    }
}

impl<T: BlockingNorStorageBusDriver> MacronixDeviceDriver<T, Blocking> {
    pub fn new_blocking(storagebusdriver: T, capacity: usize) -> Result<Self, ()> {
        Ok(Self {
            storagebusdriver,
            capacity,
            _phantom: core::marker::PhantomData,
        })
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_imxrt::init(Default::default());

    // Consider this is a storage service or file system service
    // As per the design, this service is supposed to instantiate low level bus object and configure the bus
    // and pass it to the storage device driver when creating it as a dependency injection
    // Bus drivers -
    //      1. FlexspiNorStorageBus
    //      2. SpiNorStorageBus

    let mut read_data = [0_u8; 4];
    let mut write_data = [0_u8; 4];

    let flash_config = FlexspiDeviceConfig {
        flexspi_root_clk: 48000000,
        is_sck2_enabled: false,
        // Flash size in this struct is in KB, so divide by 1KB
        flash_size_kb: 0x10000, // 64 MB
        cs_interval_unit: FlexspiCsIntervalCycleUnit::CsIntervalUnit1Cycle,
        cs_interval: 2,
        cs_hold_time: 3,
        cs_setup_time: 3,
        data_valid_time: 2,
        columnspace: 0,
        enable_word_address: false,
        awr_seq_index: 1,
        awr_seq_number: 0,
        ard_seq_index: 0,
        ard_seq_number: 0,
        ahb_write_wait_unit: FlexspiAhbWriteWaitUnit::FlexspiAhbWriteWaitUnit2ahbCycle,
        ahb_write_wait_interval: 0,
        enable_write_mask: false,
    };
    let ahb_buffer_config = FlexspiAhbBufferConfig {
        priority: 0,
        master_index: 0,
        buffer_size: 256,
        enable_prefetch: true,
    };

    let ahb_config = AhbConfig {
        enable_ahb_write_ip_rx_fifo: false,
        enable_ahb_write_ip_tx_fifo: false,
        ahb_grant_timeout_cycle: 0xff,
        ahb_bus_timeout_cycle: 0xffff,
        resume_wait_cycle: 0x20,
        buffer: [ahb_buffer_config; 8],
        enable_clear_ahb_buffer_opt: false,
        enable_read_address_opt: true,
        enable_ahb_prefetch: true,
        enable_ahb_bufferable: true,
        enable_ahb_cachable: true,
    };

    let flexspi_config = FlexspiConfig {
        rx_sample_clock: FlexspiReadSampleClock::FlexspiReadSampleClkLoopbackInternally,
        enable_sck_free_running: false,
        enable_combination: false,
        enable_doze: false, // TODO - Check back after analyzing system low power mode requirements
        enable_half_speed_access: false,
        enable_sck_b_diff_opt: false,
        enable_same_config_for_all: false,
        seq_timeout_cycle: 0xFFFF,
        ip_grant_timeout_cycle: 0xff,
        tx_watermark: 0x08,
        rx_watermark: 0x08,
        ahb_config,
    };

    let flexspi_storage = FlexspiNorStorageBus::new_blocking(
        p.FLEXSPI,       // FlexSPI peripheral
        Some(p.PIO1_11), // FlexSPI DATA 0 pin
        Some(p.PIO1_12),
        Some(p.PIO1_13),
        Some(p.PIO1_14),
        Some(p.PIO2_17),
        Some(p.PIO2_18),
        Some(p.PIO2_22),
        Some(p.PIO2_23),
        p.PIO1_29,
        p.PIO2_19,
        FlexSpiFlashPort::PortB,                         // FlexSPI port
        FlexSpiBusWidth::Octal,                          // FlexSPI bus width
        FlexSpiFlashPortDeviceInstance::DeviceInstance0, // FlexSPI device instance
    );

    // flexspi_storage.configport.configure_flexspi(&flexspi_config); // Configure the Flexspi controller

    // flexspi_storage
    //     .configport
    //     .configure_flexspi_device(&flash_config, &flexspi_config); // Configure the Flash device specific parameters like CS time, etc

    // Instanctiate the storage device driver and inject the bus driver dependency
    let mut device_driver = MacronixDeviceDriver::new_blocking(flexspi_storage, 0x4000000).unwrap();

    info!("Array before reading");
    for i in 0..4 {
        info!("data[{}]: {}", i, read_data[i]);
    }

    device_driver.read(ADDR, &mut read_data);

    info!("Array after reading");

    for i in 0..4 {
        info!("data[{}]: {}", i, read_data[i]);
    }

    device_driver.write(ADDR, &write_data);

    loop {
        Timer::after_millis(2000).await;
    }
}

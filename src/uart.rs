//! Universal Asynchronous Receiver Transmitter (UART) driver.

use core::marker::PhantomData;

use embassy_hal_internal::{into_ref, Peripheral, PeripheralRef};

use crate::gpio::{AnyPin, GpioPin as Pin};
use crate::iopctl::{DriveMode, DriveStrength, Inverter, IopctlPin, Pull, SlewRate};
use crate::pac::usart0::cfg::{Clkpol, Datalen, Loop, Paritysel as Parity, Stoplen, Syncen, Syncmst};
use crate::pac::usart0::ctl::Cc;

/// Driver move trait.
#[allow(private_bounds)]
pub trait Mode: sealed::Sealed {}

/// Blocking mode.
pub struct Blocking;
impl sealed::Sealed for Blocking {}
impl Mode for Blocking {}

/// Uart driver.
pub struct Uart<'a, M: Mode> {
    info: Info,
    tx: UartTx<'a, M>,
    rx: UartRx<'a, M>,
}

/// Uart TX driver.
pub struct UartTx<'a, M: Mode> {
    info: Info,
    _phantom: PhantomData<(&'a (), M)>,
}

/// Uart RX driver.
pub struct UartRx<'a, M: Mode> {
    info: Info,
    _phantom: PhantomData<(&'a (), M)>,
}

/// UART config
#[derive(Clone, Copy)]
pub struct Config {
    /// Baudrate of the Uart
    pub baudrate: u32,
    /// data length
    pub data_bits: Datalen,
    /// Parity
    pub parity: Parity,
    /// Stop bits
    pub stop_bits: Stoplen,
    /// Polarity of the clock
    pub clock_polarity: Clkpol,
    /// Sync/ Async operation selection
    pub operation: Syncen,
    /// Sync master/slave mode selection (only applicable in sync mode)
    pub sync_mode_master_select: Syncmst,
    /// USART continuous Clock generation enable in synchronous master mode.
    pub continuous_clock: Cc,
    /// Normal/ loopback mode
    pub loopback_mode: Loop,
}

impl Default for Config {
    /// Default configuration for single channel sampling.
    fn default() -> Self {
        Self {
            baudrate: 115_200,
            data_bits: Datalen::Bit8,
            parity: Parity::NoParity,
            stop_bits: Stoplen::Bit1,
            clock_polarity: Clkpol::FallingEdge,
            operation: Syncen::AsynchronousMode,
            sync_mode_master_select: Syncmst::Slave,
            continuous_clock: Cc::ClockOnCharacter,
            loopback_mode: Loop::Normal,
        }
    }
}

/// Uart Errors
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Read error
    Read,

    /// Buffer overflow
    Overrun,

    /// Noise error
    Noise,

    /// Framing error
    Framing,

    /// Parity error
    Parity,

    /// Failure
    Fail,

    /// Invalid argument
    InvalidArgument,

    /// Uart baud rate cannot be supported with the given clock
    UnsupportedBaudrate,

    /// RX FIFO Empty
    RxFifoEmpty,

    /// TX FIFO Full
    TxFifoFull,

    /// TX Busy
    TxBusy,
}
/// shorthand for -> Result<T>
pub type Result<T> = core::result::Result<T, Error>;

impl<'a, M: Mode> UartTx<'a, M> {
    /// Create a new blocking UART which can only send data
    /// Unidirectional Uart - Tx only
    pub fn new_blocking<T: Instance>(
        _inner: impl Peripheral<P = T> + 'a,
        tx: impl Peripheral<P = impl TxPin<T>> + 'a,
        config: Config,
    ) -> Result<Self> {
        into_ref!(_inner);
        into_ref!(tx);
        tx.as_tx();

        let mut _tx = tx.map_into();
        Uart::<Blocking>::init::<T>(Some(_tx.reborrow()), None, config)?;

        Ok(Self::new_inner::<T>())
    }

    fn new_inner<T: Instance>() -> Self {
        Self {
            info: T::info(),
            _phantom: PhantomData,
        }
    }
}

impl<'a> UartTx<'a, Blocking> {
    fn write_byte_internal(&mut self, byte: u8) -> Result<()> {
        // SAFETY: unsafe only used for .bits()
        self.info
            .regs
            .fifowr()
            .write(|w| unsafe { w.txdata().bits(u16::from(byte)) });

        Ok(())
    }

    fn blocking_write_byte(&mut self, byte: u8) -> Result<()> {
        while self.info.regs.fifostat().read().txnotfull().bit_is_clear() {}
        self.write_byte_internal(byte)
    }

    fn write_byte(&mut self, byte: u8) -> Result<()> {
        if self.info.regs.fifostat().read().txnotfull().bit_is_clear() {
            Err(Error::TxFifoFull)
        } else {
            self.write_byte_internal(byte)
        }
    }

    /// Transmit the provided buffer blocking execution until done.
    pub fn blocking_write(&mut self, buf: &[u8]) -> Result<()> {
        for x in buf {
            self.blocking_write_byte(*x)?;
        }

        Ok(())
    }

    /// Transmit the provided buffer.
    pub fn write(&mut self, buf: &[u8]) -> Result<()> {
        for x in buf {
            self.write_byte(*x)?;
        }

        Ok(())
    }

    /// Flush UART TX blocking execution until done.
    pub fn blocking_flush(&mut self) -> Result<()> {
        while self.info.regs.stat().read().txidle().bit_is_clear() {}
        Ok(())
    }

    /// Flush UART TX.
    pub fn flush(&mut self) -> Result<()> {
        if self.info.regs.stat().read().txidle().bit_is_clear() {
            Err(Error::TxBusy)
        } else {
            Ok(())
        }
    }
}

impl<'a, M: Mode> UartRx<'a, M> {
    /// Create a new blocking UART which can only receive data
    pub fn new_blocking<T: Instance>(
        _inner: impl Peripheral<P = T> + 'a,
        rx: impl Peripheral<P = impl RxPin<T>> + 'a,
        config: Config,
    ) -> Result<Self> {
        into_ref!(_inner);
        into_ref!(rx);
        rx.as_rx();

        let mut _rx = rx.map_into();
        Uart::<Blocking>::init::<T>(None, Some(_rx.reborrow()), config)?;

        Ok(Self::new_inner::<T>())
    }

    fn new_inner<T: Instance>() -> Self {
        Self {
            info: T::info(),
            _phantom: PhantomData,
        }
    }
}

impl UartRx<'_, Blocking> {
    fn read_byte_internal(&mut self) -> Result<u8> {
        if self.info.regs.fifostat().read().rxerr().bit_is_set() {
            self.info.regs.fifocfg().modify(|_, w| w.emptyrx().set_bit());
            self.info.regs.fifostat().modify(|_, w| w.rxerr().set_bit());
            Err(Error::Read)
        } else if self.info.regs.stat().read().parityerrint().bit_is_set() {
            self.info.regs.stat().modify(|_, w| w.parityerrint().clear_bit_by_one());
            Err(Error::Parity)
        } else if self.info.regs.stat().read().framerrint().bit_is_set() {
            self.info.regs.stat().modify(|_, w| w.framerrint().clear_bit_by_one());
            Err(Error::Framing)
        } else if self.info.regs.stat().read().rxnoiseint().bit_is_set() {
            self.info.regs.stat().modify(|_, w| w.rxnoiseint().clear_bit_by_one());
            Err(Error::Noise)
        } else {
            let byte = self.info.regs.fiford().read().rxdata().bits() as u8;
            Ok(byte)
        }
    }

    fn read_byte(&mut self) -> Result<u8> {
        if self.info.regs.fifostat().read().rxnotempty().bit_is_clear() {
            Err(Error::RxFifoEmpty)
        } else {
            self.read_byte_internal()
        }
    }

    fn blocking_read_byte(&mut self) -> Result<u8> {
        while self.info.regs.fifostat().read().rxnotempty().bit_is_clear() {}
        self.read_byte_internal()
    }

    /// Read from UART RX.
    pub fn read(&mut self, buf: &mut [u8]) -> Result<()> {
        for b in buf.iter_mut() {
            *b = self.read_byte()?;
        }

        Ok(())
    }

    /// Read from UART RX blocking execution until done.
    pub fn blocking_read(&mut self, buf: &mut [u8]) -> Result<()> {
        for b in buf.iter_mut() {
            *b = self.blocking_read_byte()?;
        }

        Ok(())
    }
}

impl<'a, M: Mode> Uart<'a, M> {
    /// Create a new blocking UART
    pub fn new_blocking<T: Instance>(
        _inner: impl Peripheral<P = T> + 'a,
        tx: impl Peripheral<P = impl TxPin<T>> + 'a,
        rx: impl Peripheral<P = impl RxPin<T>> + 'a,
        config: Config,
    ) -> Result<Self> {
        into_ref!(_inner);
        into_ref!(tx);
        into_ref!(rx);

        tx.as_tx();
        rx.as_rx();

        let mut tx = tx.map_into();
        let mut rx = rx.map_into();

        Self::init::<T>(Some(tx.reborrow()), Some(rx.reborrow()), config)?;

        Ok(Self {
            info: T::info(),
            tx: UartTx::new_inner::<T>(),
            rx: UartRx::new_inner::<T>(),
        })
    }

    fn init<T: Instance>(
        tx: Option<PeripheralRef<'_, AnyPin>>,
        rx: Option<PeripheralRef<'_, AnyPin>>,
        config: Config,
    ) -> Result<()> {
        // TODO - clock integration
        let clock = crate::flexcomm::Clock::Sfro;
        T::enable(clock);
        T::into_usart();

        let regs = T::info().regs;

        if tx.is_some() {
            regs.fifocfg().modify(|_, w| w.emptytx().set_bit().enabletx().enabled());

            // clear FIFO error
            regs.fifostat().write(|w| w.txerr().set_bit());
        }

        if rx.is_some() {
            regs.fifocfg().modify(|_, w| w.emptyrx().set_bit().enablerx().enabled());

            // clear FIFO error
            regs.fifostat().write(|w| w.rxerr().set_bit());
        }

        Self::set_baudrate_inner::<T>(config.baudrate)?;
        Self::set_uart_config::<T>(config);

        Ok(())
    }

    fn get_fc_freq() -> u32 {
        // Todo: Make it generic for any clock
        // Since the FC clock is hardcoded to Sfro, this freq is returned.
        // sfro : 16MHz, // ffro: 48MHz
        16_000_000
    }

    fn set_baudrate_inner<T: Instance>(baudrate: u32) -> Result<()> {
        let source_clock_hz = Self::get_fc_freq();

        if baudrate == 0 || source_clock_hz == 0 {
            return Err(Error::InvalidArgument);
        }

        let regs = T::info().regs;

        // If synchronous master mode is enabled, only configure the BRG value.
        if regs.cfg().read().syncen().is_synchronous_mode() {
            // Master
            if regs.cfg().read().syncmst().is_master() {
                // Calculate the BRG value
                let brgval = (source_clock_hz / baudrate) - 1;

                // SAFETY: unsafe only used for .bits()
                regs.brg().write(|w| unsafe { w.brgval().bits(brgval as u16) });
            }
        } else {
            // Smaller values of OSR can make the sampling position within a
            // data bit less accurate and may potentially cause more noise
            // errors or incorrect data.
            let (_, osr, brg) = (8..16).rev().fold(
                (u32::MAX, u32::MAX, u32::MAX),
                |(best_diff, best_osr, best_brg), osrval| {
                    let brgval = (source_clock_hz / ((osrval + 1) * baudrate)) - 1;
                    let diff;

                    if brgval > 65535 {
                        (best_diff, best_osr, best_brg)
                    } else {
                        // Calculate the baud rate based on the BRG value
                        let candidate = source_clock_hz / ((osrval + 1) * (brgval + 1));

                        // Calculate the difference between the
                        // current baud rate and the desired baud rate
                        diff = (candidate as i32 - baudrate as i32).unsigned_abs();

                        // Check if the current calculated difference is the best so far
                        if diff < best_diff {
                            (diff, osrval, brgval)
                        } else {
                            (best_diff, best_osr, best_brg)
                        }
                    }
                },
            );

            // Value over range
            if brg > 65535 {
                return Err(Error::UnsupportedBaudrate);
            }

            // SAFETY: unsafe only used for .bits()
            regs.osr().write(|w| unsafe { w.osrval().bits(osr as u8) });

            // SAFETY: unsafe only used for .bits()
            regs.brg().write(|w| unsafe { w.brgval().bits(brg as u16) });
        }

        Ok(())
    }

    fn set_uart_config<T: Instance>(config: Config) {
        let regs = T::info().regs;

        regs.cfg().write(|w| w.enable().disabled());

        regs.cfg().modify(|_, w| {
            w.datalen()
                .variant(config.data_bits)
                .stoplen()
                .variant(config.stop_bits)
                .paritysel()
                .variant(config.parity)
                .loop_()
                .variant(config.loopback_mode)
                .syncen()
                .variant(config.operation)
                .clkpol()
                .variant(config.clock_polarity)
        });

        regs.cfg().modify(|_, w| w.enable().enabled());
    }

    /// Deinitializes a USART instance.
    pub fn deinit(&self) -> Result<()> {
        // This function waits for TX complete, disables TX and RX, and disables the USART clock
        while self.info.regs.stat().read().txidle().bit_is_clear() {
            // When 0, indicates that the transmitter is currently in the process of sending data.
        }

        // Disable interrupts
        self.info.regs.fifointenclr().modify(|_, w| {
            w.txerr()
                .set_bit()
                .rxerr()
                .set_bit()
                .txlvl()
                .set_bit()
                .rxlvl()
                .set_bit()
        });

        // Disable dma requests
        self.info
            .regs
            .fifocfg()
            .modify(|_, w| w.dmatx().clear_bit().dmarx().clear_bit());

        // Disable peripheral
        self.info.regs.cfg().modify(|_, w| w.enable().disabled());

        Ok(())
    }
}

impl<'a> Uart<'a, Blocking> {
    /// Read from UART RX blocking execution until done.
    pub fn blocking_read(&mut self, buf: &mut [u8]) -> Result<()> {
        self.rx.blocking_read(buf)
    }

    /// Read from UART Rx.
    pub fn read(&mut self, buf: &mut [u8]) -> Result<()> {
        self.rx.read(buf)
    }

    /// Transmit the provided buffer blocking execution until done.
    pub fn blocking_write(&mut self, buf: &[u8]) -> Result<()> {
        self.tx.blocking_write(buf)
    }

    /// Transmit the provided buffer.
    pub fn write(&mut self, buf: &[u8]) -> Result<()> {
        self.tx.write(buf)
    }

    /// Flush UART TX blocking execution until done.
    pub fn blocking_flush(&mut self) -> Result<()> {
        self.tx.blocking_flush()
    }

    /// Flush UART TX.
    pub fn flush(&mut self) -> Result<()> {
        self.tx.flush()
    }

    /// Split the Uart into a transmitter and receiver, which is particularly
    /// useful when having two tasks correlating to transmitting and receiving.
    pub fn split(self) -> (UartTx<'a, Blocking>, UartRx<'a, Blocking>) {
        (self.tx, self.rx)
    }

    /// Split the Uart into a transmitter and receiver by mutable reference,
    /// which is particularly useful when having two tasks correlating to
    /// transmitting and receiving.
    pub fn split_ref(&mut self) -> (&mut UartTx<'a, Blocking>, &mut UartRx<'a, Blocking>) {
        (&mut self.tx, &mut self.rx)
    }
}

impl embedded_hal_02::serial::Read<u8> for UartRx<'_, Blocking> {
    type Error = Error;

    fn read(&mut self) -> core::result::Result<u8, nb::Error<Self::Error>> {
        let mut buf = [0; 1];

        match self.read(&mut buf) {
            Ok(_) => Ok(buf[0]),
            Err(Error::RxFifoEmpty) => Err(nb::Error::WouldBlock),
            Err(e) => Err(nb::Error::Other(e)),
        }
    }
}

impl embedded_hal_02::serial::Write<u8> for UartTx<'_, Blocking> {
    type Error = Error;

    fn write(&mut self, word: u8) -> core::result::Result<(), nb::Error<Self::Error>> {
        match self.write(&[word]) {
            Ok(_) => Ok(()),
            Err(Error::TxFifoFull) => Err(nb::Error::WouldBlock),
            Err(e) => Err(nb::Error::Other(e)),
        }
    }

    fn flush(&mut self) -> core::result::Result<(), nb::Error<Self::Error>> {
        match self.flush() {
            Ok(_) => Ok(()),
            Err(Error::TxBusy) => Err(nb::Error::WouldBlock),
            Err(e) => Err(nb::Error::Other(e)),
        }
    }
}

impl embedded_hal_02::blocking::serial::Write<u8> for UartTx<'_, Blocking> {
    type Error = Error;

    fn bwrite_all(&mut self, buffer: &[u8]) -> core::result::Result<(), Self::Error> {
        self.blocking_write(buffer)
    }

    fn bflush(&mut self) -> core::result::Result<(), Self::Error> {
        self.blocking_flush()
    }
}

impl embedded_hal_02::serial::Read<u8> for Uart<'_, Blocking> {
    type Error = Error;

    fn read(&mut self) -> core::result::Result<u8, nb::Error<Self::Error>> {
        embedded_hal_02::serial::Read::read(&mut self.rx)
    }
}

impl embedded_hal_02::serial::Write<u8> for Uart<'_, Blocking> {
    type Error = Error;

    fn write(&mut self, word: u8) -> core::result::Result<(), nb::Error<Self::Error>> {
        embedded_hal_02::serial::Write::write(&mut self.tx, word)
    }

    fn flush(&mut self) -> core::result::Result<(), nb::Error<Self::Error>> {
        embedded_hal_02::serial::Write::flush(&mut self.tx)
    }
}

impl embedded_hal_02::blocking::serial::Write<u8> for Uart<'_, Blocking> {
    type Error = Error;

    fn bwrite_all(&mut self, buffer: &[u8]) -> core::result::Result<(), Self::Error> {
        self.blocking_write(buffer)
    }

    fn bflush(&mut self) -> core::result::Result<(), Self::Error> {
        self.blocking_flush()
    }
}

impl embedded_hal_nb::serial::Error for Error {
    fn kind(&self) -> embedded_hal_nb::serial::ErrorKind {
        match *self {
            Self::Framing => embedded_hal_nb::serial::ErrorKind::FrameFormat,
            Self::Overrun => embedded_hal_nb::serial::ErrorKind::Overrun,
            Self::Parity => embedded_hal_nb::serial::ErrorKind::Parity,
            Self::Noise => embedded_hal_nb::serial::ErrorKind::Noise,
            _ => embedded_hal_nb::serial::ErrorKind::Other,
        }
    }
}

impl embedded_hal_nb::serial::ErrorType for UartRx<'_, Blocking> {
    type Error = Error;
}

impl embedded_hal_nb::serial::ErrorType for UartTx<'_, Blocking> {
    type Error = Error;
}

impl embedded_hal_nb::serial::ErrorType for Uart<'_, Blocking> {
    type Error = Error;
}

impl embedded_hal_nb::serial::Read for UartRx<'_, Blocking> {
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        let mut buf = [0; 1];

        match self.read(&mut buf) {
            Ok(_) => Ok(buf[0]),
            Err(Error::RxFifoEmpty) => Err(nb::Error::WouldBlock),
            Err(e) => Err(nb::Error::Other(e)),
        }
    }
}

impl embedded_hal_nb::serial::Write for UartTx<'_, Blocking> {
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        match self.write(&[word]) {
            Ok(_) => Ok(()),
            Err(Error::TxFifoFull) => Err(nb::Error::WouldBlock),
            Err(e) => Err(nb::Error::Other(e)),
        }
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        match self.flush() {
            Ok(_) => Ok(()),
            Err(Error::TxBusy) => Err(nb::Error::WouldBlock),
            Err(e) => Err(nb::Error::Other(e)),
        }
    }
}

impl embedded_hal_nb::serial::Read for Uart<'_, Blocking> {
    fn read(&mut self) -> core::result::Result<u8, nb::Error<Self::Error>> {
        embedded_hal_02::serial::Read::read(&mut self.rx)
    }
}

impl embedded_hal_nb::serial::Write for Uart<'_, Blocking> {
    fn write(&mut self, char: u8) -> nb::Result<(), Self::Error> {
        self.blocking_write(&[char]).map_err(nb::Error::Other)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.blocking_flush().map_err(nb::Error::Other)
    }
}

impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

impl embedded_io::ErrorType for UartRx<'_, Blocking> {
    type Error = Error;
}

impl embedded_io::ErrorType for UartTx<'_, Blocking> {
    type Error = Error;
}

impl embedded_io::ErrorType for Uart<'_, Blocking> {
    type Error = Error;
}

impl embedded_io::Read for UartRx<'_, Blocking> {
    fn read(&mut self, buf: &mut [u8]) -> core::result::Result<usize, Self::Error> {
        self.blocking_read(buf).map(|_| buf.len())
    }
}

impl embedded_io::Write for UartTx<'_, Blocking> {
    fn write(&mut self, buf: &[u8]) -> core::result::Result<usize, Self::Error> {
        self.blocking_write(buf).map(|_| buf.len())
    }

    fn flush(&mut self) -> core::result::Result<(), Self::Error> {
        self.blocking_flush()
    }
}

impl embedded_io::Read for Uart<'_, Blocking> {
    fn read(&mut self, buf: &mut [u8]) -> core::result::Result<usize, Self::Error> {
        embedded_io::Read::read(&mut self.rx, buf)
    }
}

impl embedded_io::Write for Uart<'_, Blocking> {
    fn write(&mut self, buf: &[u8]) -> core::result::Result<usize, Self::Error> {
        embedded_io::Write::write(&mut self.tx, buf)
    }

    fn flush(&mut self) -> core::result::Result<(), Self::Error> {
        embedded_io::Write::flush(&mut self.tx)
    }
}

struct Info {
    regs: &'static crate::pac::usart0::RegisterBlock,
}

trait SealedInstance {
    fn info() -> Info;
}

/// UART instancve trait.
#[allow(private_bounds)]
pub trait Instance: crate::flexcomm::IntoUsart + SealedInstance + Peripheral<P = Self> + 'static + Send {}

macro_rules! impl_instance {
    ($fc:ident, $usart:ident) => {
        impl SealedInstance for crate::peripherals::$fc {
            fn info() -> Info {
                Info {
                    regs: unsafe { &*crate::pac::$usart::ptr() },
                }
            }
        }

        impl Instance for crate::peripherals::$fc {}
    };
}

impl_instance!(FLEXCOMM0, Usart0);
impl_instance!(FLEXCOMM1, Usart1);
impl_instance!(FLEXCOMM2, Usart2);
impl_instance!(FLEXCOMM3, Usart3);
impl_instance!(FLEXCOMM4, Usart4);
impl_instance!(FLEXCOMM5, Usart5);
impl_instance!(FLEXCOMM6, Usart6);
impl_instance!(FLEXCOMM7, Usart7);

macro_rules! impl_uart_tx {
    ($piom_n:ident, $fn:ident, $fcn:ident) => {
        impl TxPin<crate::peripherals::$fcn> for crate::peripherals::$piom_n {
            fn as_tx(&self) {
                // UM11147 table 507 pg 495
                self.set_function(crate::iopctl::Function::$fn)
                    .set_pull(Pull::None)
                    .enable_input_buffer()
                    .set_slew_rate(SlewRate::Standard)
                    .set_drive_strength(DriveStrength::Normal)
                    .disable_analog_multiplex()
                    .set_drive_mode(DriveMode::PushPull)
                    .set_input_inverter(Inverter::Disabled);
            }
        }
    };
}

mod sealed {
    /// simply seal a trait
    pub trait Sealed {}
}

impl<T: Pin> sealed::Sealed for T {}

/// io configuration trait for Uart Tx configuration
pub trait TxPin<T: Instance>: Pin + sealed::Sealed + crate::Peripheral {
    /// convert the pin to appropriate function for Uart Tx  usage
    fn as_tx(&self);
}

/// io configuration trait for Uart Rx configuration
pub trait RxPin<T: Instance>: Pin + sealed::Sealed + crate::Peripheral {
    /// convert the pin to appropriate function for Uart Rx  usage
    fn as_rx(&self);
}

macro_rules! impl_uart_rx {
    ($piom_n:ident, $fn:ident, $fcn:ident) => {
        impl RxPin<crate::peripherals::$fcn> for crate::peripherals::$piom_n {
            fn as_rx(&self) {
                // UM11147 table 507 pg 495
                self.set_function(crate::iopctl::Function::$fn)
                    .set_pull(Pull::None)
                    .enable_input_buffer()
                    .set_slew_rate(SlewRate::Standard)
                    .set_drive_strength(DriveStrength::Normal)
                    .disable_analog_multiplex()
                    .set_drive_mode(DriveMode::PushPull)
                    .set_input_inverter(Inverter::Disabled);
            }
        }
    };
}

// Flexcomm0 Uart TX/Rx
impl_uart_tx!(PIO0_1, F1, FLEXCOMM0); //Tx
impl_uart_rx!(PIO0_2, F1, FLEXCOMM0); //Rx
impl_uart_tx!(PIO3_1, F5, FLEXCOMM0); //Tx
impl_uart_rx!(PIO3_2, F5, FLEXCOMM0); //Rx

// Flexcomm1 Uart TX/Rx
impl_uart_tx!(PIO0_8, F1, FLEXCOMM1); //Tx
impl_uart_rx!(PIO0_9, F1, FLEXCOMM1); //Rx
impl_uart_tx!(PIO7_26, F1, FLEXCOMM1); //Tx
impl_uart_rx!(PIO7_27, F1, FLEXCOMM1); //Rx

// Flexcomm2 Uart Tx/Rx
impl_uart_tx!(PIO0_15, F1, FLEXCOMM2); //Tx
impl_uart_rx!(PIO0_16, F1, FLEXCOMM2); //Rx
impl_uart_tx!(PIO7_30, F5, FLEXCOMM2); //Tx
impl_uart_rx!(PIO7_31, F5, FLEXCOMM2); //Rx

// Flexcomm3 Uart Tx/Rx
impl_uart_tx!(PIO0_22, F1, FLEXCOMM3); //Tx
impl_uart_rx!(PIO0_23, F1, FLEXCOMM3); //Rx

// Flexcomm4 Uart Tx/Rx
impl_uart_tx!(PIO0_29, F1, FLEXCOMM4); //Tx
impl_uart_rx!(PIO0_30, F1, FLEXCOMM4); //Rx

// Flexcomm5 Uart Tx/Rx
impl_uart_tx!(PIO1_4, F1, FLEXCOMM5); //Tx
impl_uart_rx!(PIO1_5, F1, FLEXCOMM5); //Rx
impl_uart_tx!(PIO3_16, F5, FLEXCOMM5); //Tx
impl_uart_rx!(PIO3_17, F5, FLEXCOMM5); //Rx

// Flexcomm6 Uart Tx/Rx
impl_uart_tx!(PIO3_26, F1, FLEXCOMM6); //Tx
impl_uart_rx!(PIO3_27, F1, FLEXCOMM6); //Rx

// Flexcomm7 Uart Tx/Rx
impl_uart_tx!(PIO4_1, F1, FLEXCOMM7); //Tx
impl_uart_rx!(PIO4_2, F1, FLEXCOMM7); //Rx

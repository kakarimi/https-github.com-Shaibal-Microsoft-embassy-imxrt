/// I2C Master Driver
use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use embassy_futures::select::{select, Either};
use embassy_hal_internal::into_ref;

use super::{
    Async, Blocking, Error, Info, Instance, InterruptHandler, MasterDma, Mode, Result, SclPin, SdaPin, TransferError,
    I2C_WAKERS,
};
use crate::interrupts::interrupt::typelevel::Interrupt;
use crate::{dma, interrupt, Peripheral};

/// Bus speed (nominal SCL, no clock stretching)
pub enum Speed {
    /// 100 kbit/s
    Standard,

    /// 400 kbit/s
    Fast,

    /// 1 Mbit/s
    FastPlus,

    /// 3.4Mbit/s only available for slave devices
    High,
}

/// use `FCn` as I2C Master controller
pub struct I2cMaster<'a, M: Mode> {
    info: Info,
    _phantom: PhantomData<M>,
    dma_ch: Option<dma::channel::Channel<'a>>,
}

impl<'a, M: Mode> I2cMaster<'a, M> {
    fn new_inner<T: Instance>(
        _bus: impl Peripheral<P = T> + 'a,
        scl: impl Peripheral<P = impl SclPin<T>> + 'a,
        sda: impl Peripheral<P = impl SdaPin<T>> + 'a,
        // TODO - integrate clock APIs to allow dynamic freq selection | clock: crate::flexcomm::Clock,
        speed: Speed,
        dma_ch: Option<dma::channel::Channel<'a>>,
    ) -> Result<Self> {
        into_ref!(_bus);
        into_ref!(scl);
        into_ref!(sda);

        sda.as_sda();
        scl.as_scl();

        let info = T::info();
        let regs = info.regs;

        // this check should be redundant with T::set_mode()? above

        // rates taken assuming SFRO:
        //
        //  7 => 403.3 kHz
        //  9 => 322.6 kHz
        // 12 => 247.8 kHz
        // 16 => 198.2 kHz
        // 18 => 166.6 Khz
        // 22 => 142.6 kHz
        // 30 => 100.0 kHz
        match speed {
            // 100 kHz
            Speed::Standard => {
                regs.clkdiv().write(|w|
                // SAFETY: only unsafe due to .bits usage
                unsafe { w.divval().bits(30) });
            }

            // 400 kHz
            Speed::Fast => {
                regs.clkdiv().write(|w|
                // SAFETY: only unsafe due to .bits usage
                unsafe { w.divval().bits(7) });
            }

            _ => return Err(Error::UnsupportedConfiguration),
        }

        regs.msttime().write(|w|
            // SAFETY: only unsafe due to .bits usage
            unsafe { w.mstsclhigh().bits(0).mstscllow().bits(1) });

        regs.intenset().write(|w|
                // SAFETY: only unsafe due to .bits usage
                unsafe { w.bits(0) });

        regs.cfg().write(|w| w.msten().set_bit());

        Ok(Self {
            info,
            _phantom: PhantomData,
            dma_ch,
        })
    }

    fn check_for_bus_errors(&self) -> Result<()> {
        let i2cregs = self.info.regs;

        if i2cregs.stat().read().mstarbloss().is_arbitration_loss() {
            Err(TransferError::ArbitrationLoss.into())
        } else if i2cregs.stat().read().mstststperr().is_error() {
            Err(TransferError::StartStopError.into())
        } else {
            Ok(())
        }
    }
}

impl<'a> I2cMaster<'a, Blocking> {
    /// use flexcomm fc with Pins scl, sda as an I2C Master bus, configuring to speed and pull
    pub fn new_blocking<T: Instance>(
        fc: impl Peripheral<P = T> + 'a,
        scl: impl Peripheral<P = impl SclPin<T>> + 'a,
        sda: impl Peripheral<P = impl SdaPin<T>> + 'a,
        // TODO - integrate clock APIs to allow dynamic freq selection | clock: crate::flexcomm::Clock,
        speed: Speed,
    ) -> Result<Self> {
        // TODO - clock integration
        let clock = crate::flexcomm::Clock::Sfro;
        T::enable(clock);
        T::into_i2c();

        let this = Self::new_inner::<T>(fc, scl, sda, speed, None)?;

        Ok(this)
    }

    fn start(&mut self, address: u8, is_read: bool) -> Result<()> {
        let i2cregs = self.info.regs;

        self.poll_ready()?;

        // cannot start if the the bus is already busy
        if i2cregs.stat().read().mstpending().is_in_progress() {
            return Err(TransferError::OtherBusError.into());
        }

        i2cregs.mstdat().write(|w|
            // SAFETY: only unsafe due to .bits usage
            unsafe { w.data().bits(address << 1 | u8::from(is_read)) });

        i2cregs.mstctl().write(|w| w.mststart().set_bit());

        self.poll_ready()?;

        if i2cregs.stat().read().mststate().is_nack_address() {
            // STOP bit to complete the attempted transfer
            self.stop()?;

            return Err(TransferError::AddressNack.into());
        }

        if is_read && !i2cregs.stat().read().mststate().is_receive_ready() {
            return Err(TransferError::ReadFail.into());
        }

        if !is_read && !i2cregs.stat().read().mststate().is_transmit_ready() {
            return Err(TransferError::WriteFail.into());
        }

        self.check_for_bus_errors()
    }

    fn read_no_stop(&mut self, address: u8, read: &mut [u8]) -> Result<()> {
        let i2cregs = self.info.regs;

        // read of 0 size is not allowed according to i2c spec
        if read.is_empty() {
            return Err(TransferError::OtherBusError.into());
        }

        self.start(address, true)?;

        let read_len = read.len();

        for (i, r) in read.iter_mut().enumerate() {
            self.poll_ready()?;

            // check transmission continuity
            if !i2cregs.stat().read().mststate().is_receive_ready() {
                return Err(TransferError::ReadFail.into());
            }

            self.check_for_bus_errors()?;

            *r = i2cregs.mstdat().read().data().bits();

            // continue after ACK until last byte
            if i < read_len - 1 {
                i2cregs.mstctl().write(|w| w.mstcontinue().set_bit());
            }
        }

        Ok(())
    }

    fn write_no_stop(&mut self, address: u8, write: &[u8]) -> Result<()> {
        // Procedure from 24.3.1.1 pg 545
        let i2cregs = self.info.regs;

        self.start(address, false)?;

        for byte in write {
            i2cregs.mstdat().write(|w|
                // SAFETY: unsafe only due to .bits usage
                unsafe { w.data().bits(*byte) });

            i2cregs.mstctl().write(|w| w.mstcontinue().set_bit());

            self.poll_ready()?;
            self.check_for_bus_errors()?;
        }

        Ok(())
    }

    fn stop(&mut self) -> Result<()> {
        // Procedure from 24.3.1.1 pg 545
        let i2cregs = self.info.regs;

        i2cregs.mstctl().write(|w| w.mststop().set_bit());
        self.poll_ready()?;
        self.check_for_bus_errors()?;

        // ensure return to idle state for bus (no stuck SCL/SDA lines)
        if i2cregs.stat().read().mststate().is_idle() {
            Ok(())
        } else {
            Err(TransferError::OtherBusError.into())
        }
    }

    fn poll_ready(&mut self) -> Result<()> {
        while self.info.regs.stat().read().mstpending().is_in_progress() {}

        Ok(())
    }
}

impl<'a> I2cMaster<'a, Async> {
    /// use flexcomm fc with Pins scl, sda as an I2C Master bus, configuring to speed and pull
    pub fn new_async<T: Instance>(
        fc: impl Peripheral<P = T> + 'a,
        scl: impl Peripheral<P = impl SclPin<T>> + 'a,
        sda: impl Peripheral<P = impl SdaPin<T>> + 'a,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'a,
        // TODO - integrate clock APIs to allow dynamic freq selection | clock: crate::flexcomm::Clock,
        speed: Speed,
        dma_ch: impl Peripheral<P = impl MasterDma<T>> + 'a,
    ) -> Result<Self> {
        // TODO - clock integration
        let clock = crate::flexcomm::Clock::Sfro;
        T::enable(clock);
        T::into_i2c();

        let ch = dma::Dma::reserve_channel(dma_ch);
        let this = Self::new_inner::<T>(fc, scl, sda, speed, Some(ch))?;

        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        Ok(this)
    }

    async fn start(&mut self, address: u8, is_read: bool) -> Result<()> {
        let i2cregs = self.info.regs;

        self.wait_on(
            |me| {
                let stat = me.info.regs.stat().read();

                if stat.mstpending().is_pending() {
                    Poll::Ready(Ok::<(), Error>(()))
                } else if stat.mstarbloss().is_arbitration_loss() {
                    Poll::Ready(Err(TransferError::ArbitrationLoss.into()))
                } else if stat.mstststperr().is_error() {
                    Poll::Ready(Err(TransferError::StartStopError.into()))
                } else {
                    Poll::Pending
                }
            },
            |me| {
                me.info.regs.intenset().write(|w| {
                    w.mstpendingen()
                        .set_bit()
                        .mstarblossen()
                        .set_bit()
                        .mstststperren()
                        .set_bit()
                });
            },
        )
        .await?;

        i2cregs.mstdat().write(|w|
            // SAFETY: only unsafe due to .bits usage
            unsafe { w.data().bits(address << 1 | u8::from(is_read)) });

        i2cregs.mstctl().write(|w| w.mststart().set_bit());

        self.wait_on(
            |me| {
                let stat = me.info.regs.stat().read();

                if is_read && stat.mststate().is_receive_ready() || !is_read && stat.mststate().is_transmit_ready() {
                    Poll::Ready(Ok(()))
                } else if stat.mststate().is_nack_address() {
                    Poll::Ready(Err(TransferError::AddressNack.into()))
                } else if is_read && stat.mstpending().is_pending() && !stat.mststate().is_receive_ready() {
                    Poll::Ready(Err(TransferError::ReadFail.into()))
                } else if !is_read && stat.mstpending().is_pending() && !stat.mststate().is_transmit_ready() {
                    Poll::Ready(Err(TransferError::WriteFail.into()))
                } else if stat.mstststperr().is_error() {
                    Poll::Ready(Err(TransferError::StartStopError.into()))
                } else {
                    Poll::Pending
                }
            },
            |me| {
                me.info.regs.intenset().write(|w| {
                    w.mstpendingen()
                        .set_bit()
                        .mstarblossen()
                        .set_bit()
                        .mstststperren()
                        .set_bit()
                });
            },
        )
        .await
    }

    async fn read_no_stop(&mut self, address: u8, read: &mut [u8]) -> Result<()> {
        let i2cregs = self.info.regs;

        // read of 0 size is not allowed according to i2c spec
        if read.is_empty() {
            return Err(TransferError::OtherBusError.into());
        }

        self.start(address, true).await?;

        let transfer = dma::transfer::Transfer::new_read(
            self.dma_ch.as_mut().unwrap(),
            i2cregs.mstdat().as_ptr() as *mut u8,
            read,
            Default::default(),
        );

        // According to sections 24.7.7.1 and 24.7.7.2, we should
        // first program the DMA channel for carrying out a transfer
        // and only then set MSTDMA bit.
        //
        // Additionally, at this point we know the slave has
        // acknowledged the address.
        i2cregs.mstctl().write(|w| w.mstdma().enabled());

        let res = select(
            transfer,
            poll_fn(|cx| {
                I2C_WAKERS[self.info.index].register(cx.waker());

                i2cregs.intenset().write(|w| {
                    w.mstpendingen()
                        .set_bit()
                        .mstarblossen()
                        .set_bit()
                        .mstststperren()
                        .set_bit()
                });

                let stat = i2cregs.stat().read();

                if stat.mststate().is_receive_ready() {
                    Poll::Ready(Ok::<(), Error>(()))
                } else if stat.mstarbloss().is_arbitration_loss() {
                    Poll::Ready(Err(TransferError::ArbitrationLoss.into()))
                } else if stat.mstststperr().is_error() {
                    Poll::Ready(Err(TransferError::StartStopError.into()))
                } else {
                    Poll::Pending
                }
            }),
        )
        .await;

        i2cregs.mstctl().write(|w| w.mstdma().disabled());

        if let Either::Second(e) = res {
            e?;
        }

        self.wait_on(
            |me| {
                let stat = me.info.regs.stat().read();

                if stat.mstpending().is_pending() {
                    Poll::Ready(Ok::<(), Error>(()))
                } else if stat.mstarbloss().is_arbitration_loss() {
                    Poll::Ready(Err(TransferError::ArbitrationLoss.into()))
                } else if stat.mstststperr().is_error() {
                    Poll::Ready(Err(TransferError::StartStopError.into()))
                } else {
                    Poll::Pending
                }
            },
            |me| {
                me.info.regs.intenset().write(|w| {
                    w.mstpendingen()
                        .set_bit()
                        .mstarblossen()
                        .set_bit()
                        .mstststperren()
                        .set_bit()
                });
            },
        )
        .await
    }

    async fn write_no_stop(&mut self, address: u8, write: &[u8]) -> Result<()> {
        // Procedure from 24.3.1.1 pg 545
        let i2cregs = self.info.regs;

        self.start(address, false).await?;

        if write.is_empty() {
            return Ok(());
        }

        let transfer = dma::transfer::Transfer::new_write(
            self.dma_ch.as_mut().unwrap(),
            write,
            i2cregs.mstdat().as_ptr() as *mut u8,
            Default::default(),
        );

        // According to sections 24.7.7.1 and 24.7.7.2, we should
        // first program the DMA channel for carrying out a transfer
        // and only then set MSTDMA bit.
        //
        // Additionally, at this point we know the slave has
        // acknowledged the address.
        i2cregs.mstctl().write(|w| w.mstdma().enabled());

        let res = select(
            transfer,
            poll_fn(|cx| {
                I2C_WAKERS[self.info.index].register(cx.waker());

                i2cregs.intenset().write(|w| {
                    w.mstpendingen()
                        .set_bit()
                        .mstarblossen()
                        .set_bit()
                        .mstststperren()
                        .set_bit()
                });

                let stat = i2cregs.stat().read();

                if stat.mststate().is_transmit_ready() {
                    Poll::Ready(Ok::<(), Error>(()))
                } else if stat.mstarbloss().is_arbitration_loss() {
                    Poll::Ready(Err(TransferError::ArbitrationLoss.into()))
                } else if stat.mstststperr().is_error() {
                    Poll::Ready(Err(TransferError::StartStopError.into()))
                } else {
                    Poll::Pending
                }
            }),
        )
        .await;

        i2cregs.mstctl().write(|w| w.mstdma().disabled());

        if let Either::Second(e) = res {
            e?;
        }

        self.wait_on(
            |me| {
                let stat = me.info.regs.stat().read();

                if stat.mstpending().is_pending() {
                    Poll::Ready(Ok::<(), Error>(()))
                } else if stat.mstarbloss().is_arbitration_loss() {
                    Poll::Ready(Err(TransferError::ArbitrationLoss.into()))
                } else if stat.mstststperr().is_error() {
                    Poll::Ready(Err(TransferError::StartStopError.into()))
                } else {
                    Poll::Pending
                }
            },
            |me| {
                me.info.regs.intenset().write(|w| {
                    w.mstpendingen()
                        .set_bit()
                        .mstarblossen()
                        .set_bit()
                        .mstststperren()
                        .set_bit()
                });
            },
        )
        .await
    }

    async fn stop(&mut self) -> Result<()> {
        // Procedure from 24.3.1.1 pg 545
        let i2cregs = self.info.regs;

        if i2cregs.stat().read().mstpending().is_in_progress() {
            return Err(TransferError::StartStopError.into());
        }

        i2cregs.mstctl().write(|w| w.mststop().set_bit());

        self.wait_on(
            |me| {
                let stat = me.info.regs.stat().read();

                if stat.mststate().is_idle() {
                    Poll::Ready(Ok(()))
                } else if stat.mstarbloss().is_arbitration_loss() {
                    Poll::Ready(Err(TransferError::ArbitrationLoss.into()))
                } else if stat.mstststperr().is_error() {
                    Poll::Ready(Err(TransferError::StartStopError.into()))
                } else {
                    Poll::Pending
                }
            },
            |me| {
                me.info.regs.intenset().write(|w| {
                    w.mstpendingen()
                        .set_bit()
                        .mstarblossen()
                        .set_bit()
                        .mstststperren()
                        .set_bit()
                });
            },
        )
        .await
    }

    /// Calls `f` to check if we are ready or not.
    /// If not, `g` is called once the waker is set (to eg enable the required interrupts).
    async fn wait_on<F, U, G>(&mut self, mut f: F, mut g: G) -> U
    where
        F: FnMut(&mut Self) -> Poll<U>,
        G: FnMut(&mut Self),
    {
        poll_fn(|cx| {
            let r = f(self);

            if r.is_pending() {
                I2C_WAKERS[self.info.index].register(cx.waker());

                g(self);
            }

            r
        })
        .await
    }
}

/// Error Types for I2C communication
impl embedded_hal_1::i2c::Error for Error {
    fn kind(&self) -> embedded_hal_1::i2c::ErrorKind {
        match *self {
            Self::UnsupportedConfiguration => embedded_hal_1::i2c::ErrorKind::Other,
            Self::Transfer(e) => match e {
                TransferError::Timeout => embedded_hal_1::i2c::ErrorKind::Other,
                TransferError::ReadFail | TransferError::WriteFail => {
                    embedded_hal_1::i2c::ErrorKind::NoAcknowledge(embedded_hal_1::i2c::NoAcknowledgeSource::Data)
                }
                TransferError::AddressNack => {
                    embedded_hal_1::i2c::ErrorKind::NoAcknowledge(embedded_hal_1::i2c::NoAcknowledgeSource::Address)
                }
                TransferError::ArbitrationLoss => embedded_hal_1::i2c::ErrorKind::ArbitrationLoss,
                TransferError::StartStopError => embedded_hal_1::i2c::ErrorKind::Bus,
                TransferError::OtherBusError => embedded_hal_1::i2c::ErrorKind::Bus,
            },
        }
    }
}

impl<M: Mode> embedded_hal_1::i2c::ErrorType for I2cMaster<'_, M> {
    type Error = Error;
}

// implement generic i2c interface for peripheral master type
impl embedded_hal_1::i2c::I2c for I2cMaster<'_, Blocking> {
    fn read(&mut self, address: u8, read: &mut [u8]) -> Result<()> {
        self.read_no_stop(address, read)?;
        self.stop()
    }

    fn write(&mut self, address: u8, write: &[u8]) -> Result<()> {
        self.write_no_stop(address, write)?;
        self.stop()
    }

    fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<()> {
        self.write_no_stop(address, write)?;
        self.read_no_stop(address, read)?;
        self.stop()
    }

    fn transaction(&mut self, address: u8, operations: &mut [embedded_hal_1::i2c::Operation<'_>]) -> Result<()> {
        let needs_stop = !operations.is_empty();

        for op in operations {
            match op {
                embedded_hal_1::i2c::Operation::Read(read) => {
                    self.read_no_stop(address, read)?;
                }
                embedded_hal_1::i2c::Operation::Write(write) => {
                    self.write_no_stop(address, write)?;
                }
            }
        }

        if needs_stop {
            self.stop()?;
        }

        Ok(())
    }
}

impl embedded_hal_async::i2c::I2c<embedded_hal_async::i2c::SevenBitAddress> for I2cMaster<'_, Async> {
    async fn read(&mut self, address: u8, read: &mut [u8]) -> Result<()> {
        self.read_no_stop(address, read).await?;
        self.stop().await
    }

    async fn write(&mut self, address: u8, write: &[u8]) -> Result<()> {
        self.write_no_stop(address, write).await?;
        self.stop().await
    }

    async fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<()> {
        self.write_no_stop(address, write).await?;
        self.read_no_stop(address, read).await?;
        self.stop().await
    }

    async fn transaction(&mut self, address: u8, operations: &mut [embedded_hal_1::i2c::Operation<'_>]) -> Result<()> {
        let needs_stop = !operations.is_empty();

        for op in operations {
            match op {
                embedded_hal_1::i2c::Operation::Read(read) => {
                    self.read_no_stop(address, read).await?;
                }
                embedded_hal_1::i2c::Operation::Write(write) => {
                    self.write_no_stop(address, write).await?;
                }
            }
        }

        if needs_stop {
            self.stop().await?;
        }

        Ok(())
    }
}

//! True Random Number Generator (TRNG)

use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use embassy_futures::block_on;
use embassy_hal_internal::into_ref;
use embassy_sync::waitqueue::AtomicWaker;
use rand_core::{CryptoRng, RngCore};

use crate::clocks::{enable_and_reset, SysconPeripheral};
use crate::interrupt::typelevel::Interrupt;
use crate::{interrupt, peripherals, Peripheral};

static RNG_WAKER: AtomicWaker = AtomicWaker::new();

/// RNG ;error
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Seed error.
    SeedError,

    /// HW Error.
    HwError,

    /// Frequency Count Fail
    FreqCountFail,
}

/// RNG interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let regs = T::info().regs;
        let int_status = regs.int_status().read();

        if int_status.ent_val().bit_is_set()
            || int_status.hw_err().bit_is_set()
            || int_status.frq_ct_fail().bit_is_set()
        {
            regs.int_ctrl().modify(|_, w| {
                w.ent_val()
                    .ent_val_0()
                    .hw_err()
                    .hw_err_0()
                    .frq_ct_fail()
                    .frq_ct_fail_0()
            });
            RNG_WAKER.wake();
        }
    }
}

/// RNG driver.
pub struct Rng<'d> {
    info: Info,
    _lifetime: PhantomData<&'d ()>,
}

impl<'d> Rng<'d> {
    /// Create a new RNG driver.
    pub fn new<T: Instance>(
        _inner: impl Peripheral<P = T> + 'd,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
    ) -> Self {
        enable_and_reset::<T>();

        into_ref!(_inner);

        let mut random = Self {
            info: T::info(),
            _lifetime: PhantomData,
        };
        random.init();

        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        random
    }

    /// Reset the RNG.
    pub fn reset(&mut self) {
        self.info.regs.mctl().write(|w| w.rst_def().set_bit().prgm().set_bit());
    }

    /// Fill the given slice with random values.
    pub async fn async_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), Error> {
        // We have a total of 16 words (512 bits) of entropy at our
        // disposal. The idea here is to read all bits and copy the
        // necessary bytes to the slice.
        for chunk in dest.chunks_mut(64) {
            self.async_fill_chunk(chunk).await?;
        }

        Ok(())
    }

    async fn async_fill_chunk(&mut self, chunk: &mut [u8]) -> Result<(), Error> {
        // wait for interrupt
        let res = poll_fn(|cx| {
            // Check if already ready.
            if self.info.regs.int_status().read().ent_val().bit_is_set() {
                return Poll::Ready(Ok(()));
            }

            RNG_WAKER.register(cx.waker());

            self.info.regs.int_mask().modify(|_, w| {
                w.ent_val()
                    .ent_val_1()
                    .hw_err()
                    .hw_err_1()
                    .frq_ct_fail()
                    .frq_ct_fail_1()
            });

            let mctl = self.info.regs.mctl().read();

            // Check again if interrupt fired
            if mctl.ent_val().bit_is_set() {
                Poll::Ready(Ok(()))
            } else if mctl.err().bit_is_set() {
                Poll::Ready(Err(Error::HwError))
            } else if mctl.fct_fail().bit_is_set() {
                Poll::Ready(Err(Error::FreqCountFail))
            } else {
                Poll::Pending
            }
        })
        .await;

        let bits = self.info.regs.mctl().read();

        if bits.ent_val().bit_is_set() {
            let mut entropy = [0; 16];

            for (i, item) in entropy.iter_mut().enumerate() {
                *item = self.info.regs.ent(i).read().bits();
            }

            // Read MCTL after reading ENT15
            let _ = self.info.regs.mctl().read();

            if entropy.iter().any(|e| *e == 0) {
                return Err(Error::SeedError);
            }

            // SAFETY: entropy is the same for input and output types in
            // native endianness.
            let entropy: [u8; 64] = unsafe { core::mem::transmute(entropy) };

            // write bytes to chunk
            chunk.copy_from_slice(&entropy[..chunk.len()]);
        }

        res
    }

    fn init(&mut self) {
        // Mask all interrupts
        self.info.regs.int_mask().write(|w| {
            w.ent_val()
                .ent_val_0()
                .hw_err()
                .hw_err_0()
                .frq_ct_fail()
                .frq_ct_fail_0()
        });

        // Switch TRNG to programming mode
        self.info.regs.mctl().modify(|_, w| w.prgm().set_bit());

        // Enable interrupts
        self.info.regs.int_ctrl().write(|w| {
            w.ent_val()
                .ent_val_1()
                .hw_err()
                .hw_err_1()
                .frq_ct_fail()
                .frq_ct_fail_1()
        });

        // Switch TRNG to Run Mode
        self.info
            .regs
            .mctl()
            .modify(|_, w| w.trng_acc().set_bit().prgm().clear_bit());
    }
}

impl RngCore for Rng<'_> {
    fn next_u32(&mut self) -> u32 {
        let mut bytes = [0u8; 4];
        block_on(self.async_fill_bytes(&mut bytes)).unwrap();
        u32::from_ne_bytes(bytes)
    }

    fn next_u64(&mut self) -> u64 {
        let mut bytes = [0u8; 8];
        block_on(self.async_fill_bytes(&mut bytes)).unwrap();
        u64::from_ne_bytes(bytes)
    }

    fn fill_bytes(&mut self, dest: &mut [u8]) {
        block_on(self.async_fill_bytes(dest)).unwrap();
    }

    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), rand_core::Error> {
        self.fill_bytes(dest);
        Ok(())
    }
}

impl CryptoRng for Rng<'_> {}

struct Info {
    regs: crate::pac::Trng,
}

trait SealedInstance {
    fn info() -> Info;
}

/// RNG instance trait.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + Peripheral<P = Self> + SysconPeripheral + 'static + Send {
    /// Interrupt for this RNG instance.
    type Interrupt: interrupt::typelevel::Interrupt;
}

impl Instance for peripherals::RNG {
    type Interrupt = crate::interrupt::typelevel::RNG;
}

impl SealedInstance for peripherals::RNG {
    fn info() -> Info {
        // SAFETY: safe from single executor
        Info {
            regs: unsafe { crate::pac::Trng::steal() },
        }
    }
}

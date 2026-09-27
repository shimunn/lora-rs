use cortex_m::peripheral::NVIC;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::interrupt::InterruptExt;
use embassy_stm32::peripherals::{PB8, PC13};
use embassy_stm32::{Peri, interrupt, pac};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Timer;
use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::{ErrorType, Operation, SpiBus, SpiDevice};
use lora_phy::DelayNs;
use lora_phy::mod_params::RadioError;
use lora_phy::mod_params::RadioError::*;
use lora_phy::mod_traits::InterfaceVariant;

/// Interrupt handler.
pub struct InterruptHandler {}

impl interrupt::typelevel::Handler<interrupt::typelevel::SUBGHZ_RADIO> for InterruptHandler {
    unsafe fn on_interrupt() {
        interrupt::SUBGHZ_RADIO.disable();
        IRQ_SIGNAL.signal(());
    }
}

static IRQ_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Abstraction over the control circuitry used to switch the rf path
/// Check your PCB schematic to determine the correct implementation
pub trait RFSwitches {
    const LP_SUPPORT: bool;
    const HP_SUPPORT: bool;
    fn switch_to_tx(&mut self) -> Result<(), RadioError>;
    fn switch_to_rx(&mut self) -> Result<(), RadioError>;
    fn enable_rf(&mut self) -> Result<(), RadioError>;
    fn disable_rf(&mut self) -> Result<(), RadioError>;
}

/// RAK3172 Base Module not SIP
// https://forum.rakwireless.com/t/rak3172-internal-schematic/4557/2
pub struct RAK3172Module<'a> {
    enable_tx: Output<'a>,
    enable_rx: Output<'a>,
}

impl<'a> RAK3172Module<'a> {
    pub fn new(tx: Peri<'a, PC13>, rx: Peri<'a, PB8>) -> Self {
        Self {
            enable_tx: Output::new(tx, Level::Low, Speed::High),
            enable_rx: Output::new(rx, Level::Low, Speed::High),
        }
    }

    fn switch(&mut self, tx: bool, rx: bool) -> Result<(), RadioError> {
        self.enable_tx.set_level(tx.into());
        self.enable_rx.set_level(rx.into());
        Ok(())
    }
}

impl RFSwitches for RAK3172Module<'_> {

    const LP_SUPPORT: bool = false;

    // Supports high power output only
    const HP_SUPPORT: bool = true;

    fn switch_to_tx(&mut self) -> Result<(), RadioError> {
        self.switch(true, false)
    }

    fn switch_to_rx(&mut self) -> Result<(), RadioError> {
        self.switch(false, true)
    }

    fn enable_rf(&mut self) -> Result<(), RadioError> {
        Ok(())
    }

    fn disable_rf(&mut self) -> Result<(), RadioError> {
        self.switch(false, false)
    }
}

/// Maps rx, tx and enable to one pin each
pub struct SwitchWithEnable<const HP: bool, const LP: bool, CTRL: OutputPin> {
    pub rx: Option<CTRL>,
    pub tx: Option<CTRL>,
    pub enable: Option<CTRL>,
}

impl<const HP: bool, const LP: bool, CTRL: OutputPin> SwitchWithEnable<HP, LP, CTRL> {
    fn switch(&mut self, rx: bool, tx: bool) -> Result<(), RadioError> {
        self.rx
            .iter_mut()
            .try_for_each(|pin| pin.set_state(rx.into()).map_err(|_| RadioError::RfSwitchRx))?;
        self.tx
            .iter_mut()
            .try_for_each(|pin| pin.set_state(tx.into()).map_err(|_| RadioError::RfSwitchTx))?;
        Ok(())
    }
}
impl<const HP: bool, const LP: bool, CTRL: OutputPin> RFSwitches for SwitchWithEnable<HP, LP, CTRL> {
    const LP_SUPPORT: bool = LP;

    const HP_SUPPORT: bool = HP;

    fn switch_to_tx(&mut self) -> Result<(), RadioError> {
        self.switch(false, true)
    }

    fn switch_to_rx(&mut self) -> Result<(), RadioError> {
        self.switch(true, false)
    }

    fn enable_rf(&mut self) -> Result<(), RadioError> {
        self.enable
            .iter_mut()
            .try_for_each(|pin| pin.set_state(true.into()).map_err(|_| RadioError::RfSwitchTx))
    }

    fn disable_rf(&mut self) -> Result<(), RadioError> {
        self.enable
            .iter_mut()
            .try_for_each(|pin| pin.set_state(false.into()).map_err(|_| RadioError::RfSwitchTx))
    }
}

/// Base for the InterfaceVariant implementation for an stm32wl/sx1262 combination
pub struct Stm32wlInterfaceVariant<SW: RFSwitches> {
    use_high_power_pa: bool,
    switches: SW,
}

impl<SW: RFSwitches> Stm32wlInterfaceVariant<SW> {
    /// Create an InterfaceVariant instance for an stm32wl/sx1262 combination
    pub fn new(
        _irq: impl interrupt::typelevel::Binding<interrupt::typelevel::SUBGHZ_RADIO, InterruptHandler> + 'static,
        use_high_power_pa: bool,
        switches: SW,
    ) -> Result<Self, RadioError> {
        if use_high_power_pa && !SW::HP_SUPPORT {
            return Err(RadioError::InvalidConfiguration);
        }
        if !use_high_power_pa && !SW::LP_SUPPORT {
            return Err(RadioError::InvalidConfiguration);
        }
        interrupt::SUBGHZ_RADIO.disable();
        Ok(Self {
            use_high_power_pa,
            switches,
        })
    }
}

impl<SW> InterfaceVariant for Stm32wlInterfaceVariant<SW>
where
    SW: RFSwitches,
{
    async fn reset(&mut self, _delay: &mut impl DelayNs) -> Result<(), RadioError> {
        pac::RCC.csr().modify(|w| w.set_rfrst(true));
        pac::RCC.csr().modify(|w| w.set_rfrst(false));
        Ok(())
    }
    async fn wait_on_busy(&mut self) -> Result<(), RadioError> {
        while pac::PWR.sr2().read().rfbusys() {}
        Ok(())
    }

    async fn await_irq(&mut self) -> Result<(), RadioError> {
        // Clear pending interrupts before enabling IRQ
        NVIC::unpend(pac::Interrupt::SUBGHZ_RADIO);
        unsafe { interrupt::SUBGHZ_RADIO.enable() };
        IRQ_SIGNAL.wait().await;
        Ok(())
    }

    async fn enable_rf_switch_rx(&mut self) -> Result<(), RadioError> {
        self.switches.switch_to_rx()
    }
    async fn enable_rf_switch_tx(&mut self) -> Result<(), RadioError> {
        self.switches.switch_to_tx()
    }
    async fn disable_rf_switch(&mut self) -> Result<(), RadioError> {
        self.switches.disable_rf()
    }
}
pub struct SubghzSpiDevice<T>(pub T);

impl<T: SpiBus> ErrorType for SubghzSpiDevice<T> {
    type Error = T::Error;
}

impl<T: SpiBus> SpiDevice for SubghzSpiDevice<T> {
    async fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
        pac::PWR.subghzspicr().modify(|w| w.set_nss(false));

        let op_res = 'ops: {
            for op in operations {
                let res = match op {
                    Operation::Read(buf) => self.0.read(buf).await,
                    Operation::Write(buf) => self.0.write(buf).await,
                    Operation::Transfer(read, write) => self.0.transfer(read, write).await,
                    Operation::TransferInPlace(buf) => self.0.transfer_in_place(buf).await,
                    Operation::DelayNs(ns) => match self.0.flush().await {
                        Err(e) => Err(e),
                        Ok(()) => {
                            Timer::after_nanos((*ns) as u64).await;
                            Ok(())
                        }
                    },
                };
                if let Err(e) = res {
                    break 'ops Err(e);
                }
            }
            Ok(())
        };

        // On failure, it's important to still flush and deassert CS.
        let flush_res = self.0.flush().await;

        pac::PWR.subghzspicr().modify(|w| w.set_nss(true));

        op_res?;
        flush_res?;

        Ok(())
    }
}

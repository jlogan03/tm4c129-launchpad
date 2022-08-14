//! Status register parsers
use modular_bitfield::{prelude::*, BitfieldSpecifier};
use ufmt::derive::uDebug;

/// EMACSTATUS register parser
#[bitfield(bits = 32)]
#[derive(Clone, Copy, Eq, PartialEq)]
#[allow(missing_docs)]
struct EmacStatusBitfield {
    pub rx_engine: B1,
    pub rx_frame_controller: B1,
    _reserved0: B1,
    pub rx_fifo_write_controller: B1,
    pub rx_fifo_read_controller: B2,
    _reserved1: B1,
    pub rx_fifo_fill: B2,
    _reserved2: B6,
    pub tx_engine: B1,
    pub tx_frame_controller: B2,
    pub tx_paused: B1,
    pub tx_fifo_read_controller: B2,
    pub tx_fifo_write_controller: B1,
    _reserved3: B1,
    pub tx_fifo_not_empty: B1,
    pub tx_fifo_full: B1,
    _reserved4: B7,
}

/// Display intermediary to make the EmacStatusBitfield format in a sane way
#[derive(Clone, Copy, Debug, uDebug, Eq, PartialEq)]
#[allow(missing_docs)]
pub struct EmacStatus {
    pub rx_engine: bool,
    pub rx_frame_controller: bool,
    pub rx_fifo_write_controller: bool,
    pub rx_fifo_read_controller: u8,
    pub rx_fifo_fill: u8,
    pub tx_engine: bool,
    pub tx_frame_controller: u8,
    pub tx_paused: bool,
    pub tx_fifo_read_controller: u8,
    pub tx_fifo_write_controller: bool,
    pub tx_fifo_not_empty: bool,
    pub tx_fifo_full: bool,
}

impl EmacStatus {
    /// Parse register into debuggable format
    pub fn new(reg: u32) -> Self {
        let b = EmacStatusBitfield::from_bytes(reg.to_le_bytes());
        EmacStatus {
            rx_engine: b.rx_engine() != 0,
            rx_frame_controller: b.rx_frame_controller() != 0,
            rx_fifo_write_controller: b.rx_fifo_write_controller() != 0,
            rx_fifo_read_controller: b.rx_fifo_read_controller(),
            rx_fifo_fill: b.rx_fifo_fill(),
            tx_engine: b.tx_engine() != 0,
            tx_frame_controller: b.tx_frame_controller(),
            tx_paused: b.tx_paused() != 0,
            tx_fifo_read_controller: b.tx_fifo_read_controller(),
            tx_fifo_write_controller: b.tx_fifo_write_controller() != 0,
            tx_fifo_not_empty: b.tx_fifo_not_empty() != 0,
            tx_fifo_full: b.tx_fifo_full() != 0,
        }
    }
}

/// EMACDMARIS register parser
#[bitfield(bits = 32)]
#[derive(Clone, Copy, Eq, PartialEq)]
#[allow(missing_docs)]
struct DmaStatusBitfield {
    pub tx_interrupt: B1,
    pub tx_stopped: B1,
    pub tx_buffer_unavailable: B1,
    pub tx_jabber_timeout_error: B1,
    pub rx_buffer_overflow_error: B1,
    pub tx_buffer_underflow_error: B1,
    pub rx_interrupt: B1,
    pub rx_buffer_unavailable: B1,
    pub rx_stopped: B1,
    pub rx_watchdog_timeout: B1,
    pub tx_early_interrupt: B1,
    _reserved0: B2,
    pub fatal_bus_error: B1,
    pub rx_early_interrupt: B1,
    pub abnormal_interrupt_summary: B1,
    pub normal_interrupt_summary: B1,
    pub rx_process_state: DmaRxProcessState,
    pub tx_process_state: DmaTxProcessState,
    pub access_error: DmaAccessError,
    _reserved1: B1,
    pub mac_mmc_interrupt: B1,
    pub mac_pmt_interrupt: B1,
    pub timestamp_interrupt: B1,
    _reserved2: B2,
}

/// Display intermediary to make the EmacStatusBitfield format in a sane way
#[derive(Clone, Copy, Debug, uDebug, Eq, PartialEq)]
#[allow(missing_docs)]
pub struct DmaStatus {
    pub tx_interrupt: bool,
    pub tx_stopped: bool,
    pub tx_buffer_unavailable: bool,
    pub tx_jabber_timeout_error: bool,
    pub rx_buffer_overflow_error: bool,
    pub tx_buffer_underflow_error: bool,
    pub rx_interrupt: bool,
    pub rx_buffer_unavailable: bool,
    pub rx_stopped: bool,
    pub rx_watchdog_timeout: bool,
    pub tx_early_interrupt: bool,
    pub fatal_bus_error: bool,
    pub rx_early_interrupt: bool,
    pub abnormal_interrupt_summary: bool,
    pub normal_interrupt_summary: bool,
    pub rx_process_state: DmaRxProcessState,
    pub tx_process_state: DmaTxProcessState,
    pub access_error: DmaAccessError,
    pub mac_mmc_interrupt: bool,
    pub mac_pmt_interrupt: bool,
    pub timestamp_interrupt: bool,
}

impl DmaStatus {
    /// Parse register into debuggable format
    pub fn new(reg: u32) -> Self {
        let b = DmaStatusBitfield::from_bytes(reg.to_le_bytes());
        DmaStatus {
            tx_interrupt: b.tx_interrupt() != 0,
            tx_stopped: b.tx_stopped() != 0,
            tx_buffer_unavailable: b.tx_buffer_unavailable() != 0,
            tx_jabber_timeout_error: b.tx_jabber_timeout_error() != 0,
            rx_buffer_overflow_error: b.rx_buffer_overflow_error() != 0,
            tx_buffer_underflow_error: b.tx_buffer_underflow_error() != 0,
            rx_interrupt: b.rx_interrupt() != 0,
            rx_buffer_unavailable: b.rx_buffer_unavailable() != 0,
            rx_stopped: b.rx_stopped() != 0,
            rx_watchdog_timeout: b.rx_watchdog_timeout() != 0,
            tx_early_interrupt: b.tx_early_interrupt() != 0,
            fatal_bus_error: b.fatal_bus_error() != 0,
            rx_early_interrupt: b.rx_early_interrupt() != 0,
            abnormal_interrupt_summary: b.abnormal_interrupt_summary() != 0,
            normal_interrupt_summary: b.normal_interrupt_summary() != 0,
            rx_process_state: b.rx_process_state(),
            tx_process_state: b.tx_process_state(),
            access_error: b.access_error(),
            mac_mmc_interrupt: b.mac_mmc_interrupt() != 0,
            mac_pmt_interrupt: b.mac_pmt_interrupt() != 0,
            timestamp_interrupt: b.timestamp_interrupt() != 0,
        }
    }
}

/// EMACDMARIS process status types for RS field
#[derive(BitfieldSpecifier, Clone, Copy, Debug, uDebug, Eq, PartialEq)]
#[bits = 3]
#[allow(missing_docs)]
pub enum DmaRxProcessState {
    Stopped = 0,
    RunningFetchingDescriptor = 1,
    _Reserved = 2,
    RunningWaiting = 3,
    SuspendedDescriptorUnavailable = 4,
    RunningClosingDescriptor = 5,
    RunningWritingTimestamp = 6,
    RunningTransferringToHost = 7,
}

/// EMACDMARIS process status types for TS field
#[derive(BitfieldSpecifier, Clone, Copy, Debug, uDebug, Eq, PartialEq)]
#[bits = 3]
#[allow(missing_docs)]
pub enum DmaTxProcessState {
    Stopped = 0,
    RunningFetchingDescriptor = 1,
    RunningWaiting = 2,
    RunningReadingFromHost = 3,
    RunningWritingTimestamp = 4,
    _Reserved = 5,
    SuspendedDescriptorUnavailable = 6,
    RunningClosingDescriptor = 7,
}

/// EMACDMARIS error types for AE field
#[derive(BitfieldSpecifier, Clone, Copy, Debug, uDebug, Eq, PartialEq)]
#[bits = 3]
#[allow(missing_docs)]
pub enum DmaAccessError {
    RxDmaDataWrite = 0,
    _Reserved1 = 1,
    _Reserved2 = 2,
    TxDmaDataRead = 3,
    RxDmaDescriptorWrite = 4,
    TxDmaDescriptorWrite = 5,
    RxDmaDescriptorRead = 6,
    TxDmaDescriptorRead = 7,
}

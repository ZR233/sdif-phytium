use core::{
    hint::spin_loop,
    ptr::{null, slice_from_raw_parts, slice_from_raw_parts_mut},
    sync::atomic::{fence, Ordering},
    time::Duration,
};

use log::{debug, error};
use tock_registers::{
    fields::FieldValue,
    interfaces::{ReadWriteable, Readable, Writeable},
    register_bitfields, register_structs,
    registers::*,
};

use crate::{
    define::{
        CmdDataFlag::{self, SWITCH_VOLTAGE},
        SdifCmdData, SdifData,
    },
    err::{Error, Result},
    Config, KFun, TransMode,
};

const TIMEOUT: usize = 50000;
pub const MAX_FIFO_CNT: u32 = 0x800;

register_structs! {
    pub SdRegister {
       // Control register: read-write
        (0x000 => cr: ReadWrite<u32, Cntrl::Register>),
        // Power enable register: read-write
        (0x004 => pwren: ReadWrite<u32>),
        // Clock divider register: read-write
        (0x008 => clkdiv: ReadWrite<u32>),
        (0x00C => _rsv),
        // Card clock enable register: read-write
        (0x010 => clkena: ReadWrite<u32, Clkena::Register>),
        // Timeout register: read-write
        (0x014 => tmout: ReadWrite<u32, Tmout::Register>),
        // Card type register: read-write
        (0x018 => ctype: ReadWrite<u32, CType::Register>),
        // Block size register: read-write
        (0x01C => blksiz: ReadWrite<u32>),
        // Byte count register: read-write
        (0x020 => bytcnt: ReadWrite<u32>),
        // Interrupt mask register: read-write
        (0x024 => int_mask: ReadWrite<u32, IntMask::Register>),
        // Command argument register: read-write
        (0x028 => cmdarg: ReadWrite<u32>),
        // Command register: read-write
        (0x02C => cmd: ReadWrite<u32, Cmd::Register>),
        // Response register 0: read-only
        (0x030 => resp0: ReadOnly<u32>),
        // Response register 1: read-only
        (0x034 => resp1: ReadOnly<u32>),
        // Response register 2: read-only
        (0x038 => resp2: ReadOnly<u32>),
        // Response register 3: read-only
        (0x03C => resp3: ReadOnly<u32>),
        // Masked interrupt status register: read-only
        (0x040 => masked_ints: ReadOnly<u32>),
        // Raw interrupt status register: read-only
        (0x044 => raw_ints: ReadWrite<u32>),
        // Controller status register: read-only
        (0x048 => status: ReadOnly<u32, Status::Register>),
        // FIFO threshold register: read-write
        (0x04C => fifoth: ReadWrite<u32, Fifoth::Register>),
        // Card detect register: read-only
        (0x050 => card_detect: ReadOnly<u32>),
        // Write protection register: read-only
        (0x054 => card_write_prt: ReadOnly<u32>),
        // Clock monitor register: read-only
        (0x058 => cksts: ReadOnly<u32>),
        // Controller-card transfer byte count: read-only
        (0x05C => tran_crd_cnt_mx: ReadOnly<u32>),
        // Memory-FIFO transfer byte count: read-only
        (0x060 => tran_fifo_cnt_mx: ReadOnly<u32>),
        // Debounce configuration register: read-write
        (0x064 => debnce: ReadWrite<u32>),
        // User ID register: read-only
        (0x068 => uid: ReadOnly<u32>),
        // Controller version register: read-only
        (0x06C => vid: ReadOnly<u32>),
        (0x070 => _rsv1),
        // UHS-1 register: read-write
        (0x074 => uhs_reg: ReadWrite<u32, UhsReg::Register>),
        // Card reset register: read-write
        (0x078 => card_reset: ReadWrite<u32>),
        (0x07C => _rsv7),
        // Bus mode register: read-write
        (0x080 => bus_mode_reg: ReadWrite<u32, BusMode::Register>),
        (0x084 => _rsv2),
        // Descriptor list start address register low: read-write
        (0x088 => desc_list_star_reg_l: ReadWrite<u32>),
        // Descriptor list start address register high: read-write
        (0x08C => desc_list_star_reg_u: ReadWrite<u32>),
        // Internal DMAC status register: read-only
        (0x090 => status_reg: ReadWrite<u32>),
        // Interrupt enable register: read-write
        (0x094 => intr_en_reg: ReadWrite<u32>),
        // Current host descriptor address register low: read-only
        (0x098 => cur_desc_addr_reg_l: ReadOnly<u32>),
        // Current host descriptor address register high: read-only
        (0x09C => cur_desc_addr_reg_u: ReadOnly<u32>),
        // Current buffer descriptor address register low: read-only
        (0x0A0 => cur_buf_addr_reg_l: ReadOnly<u32>),
        // Current buffer descriptor address register high: read-only
        (0x0A4 => cur_buf_addr_reg_u: ReadOnly<u32>),
        (0x0A8 => _rsv3),
        // Card threshold control register: read-write
        (0x100 => cardthctl: ReadWrite<u32, Cardthctl::Register>),
        (0x104 => _rsv4),
        // UHS register extension: read-write
        (0x108 => clksrc: ReadWrite<u32, ClkSrc::Register>),
        (0x10C => _rsv5),
        // Phase shift enable register: read-write
        (0x110 => enable_shift: ReadWrite<u32>),
        (0x114 => _rsv6),
        // Data FIFO register: read-write
        (0x200 => data: ReadWrite<u32>),
        // End of register definitions
        (0x204 => @END),
    }
}

register_bitfields! [
    u32,
    Cntrl[
        USE_INTERNAL_DMAC      OFFSET(25) NUMBITS(1)  [],
        ENABLE_OD_PULLUP       OFFSET(24) NUMBITS(1)  [],
        CARD_VOLTAGE_B         OFFSET(20) NUMBITS(4)  [],
        CARD_VOLTAGE_A         OFFSET(16) NUMBITS(4)  [],
        ENDIAN                 OFFSET(11) NUMBITS(1)  [],
        SEND_AUTO_STOP_CCSD    OFFSET(10) NUMBITS(1)  [],
        SEND_CCSD              OFFSET(9)  NUMBITS(1)  [],
        ABORT_READ_DATA        OFFSET(8)  NUMBITS(1)  [],
        SEND_IRQ_RESPONSE      OFFSET(7)  NUMBITS(1)  [],
        READ_WAIT              OFFSET(6)  NUMBITS(1)  [],
        RES_DMA_ENABLE             OFFSET(5)  NUMBITS(1)  [],
        INT_ENABLE             OFFSET(4)  NUMBITS(1)  [],
        DMA_RESET              OFFSET(2)  NUMBITS(1)  [],
        FIFO_RESET             OFFSET(1)  NUMBITS(1)  [],
        CONTROLLER_RESET       OFFSET(0)  NUMBITS(1)  [],
    ],
    IntMask[
        /// SDIO interrupt 中断
        SDIO_INT_MASK_CARD0 OFFSET(16)  NUMBITS(1)  [],
        /// 读写结束位错误/写未收到 CRC 中断
        EBE_INT_MASK OFFSET(15) NUMBITS(1) [],
        /// Auto command 完成中断
        ACD_INT_MASK OFFSET(14) NUMBITS(1) [],
        /// 起始位错误/busy 撤销中断
        SBE_BCI_INT_MASK OFFSET(13) NUMBITS(1) [],
        /// 硬件锁存中断
        HLE_INT_MASK OFFSET(12) NUMBITS(1) [],
        /// FIFO 上下溢中断
        FRUN_INT_MASK OFFSET(11) NUMBITS(1) [],
        /// 数据 starv/电源切换中断
        HTO_INT_MASK OFFSET(10) NUMBITS(1) [],
        /// 数据读超时中断
        DRTO_INT_MASK OFFSET(9) NUMBITS(1) [],
        /// 响应超时中断
        RTO_INT_MASK OFFSET(8) NUMBITS(1) [],
        /// 数据 CRC 校验错误中断
        DCRC_INT_MASK OFFSET(7) NUMBITS(1) [],
        /// 响应 CRC 错误中断
        RCRC_INT_MASK OFFSET(6) NUMBITS(1) [],
        /// 接收 FIFO 请求中断
        RXDR_INT_MASK OFFSET(5) NUMBITS(1) [],
        /// 发送 FIFO 请求中断
        TXDR_INT_MASK OFFSET(4) NUMBITS(1) [],
        /// Data transfer over (DTO) interrupt enable
        DTO_INT_MASK OFFSET(3) NUMBITS(1) [],
        /// 命令传输完成中断
        CMD_INT_MASK OFFSET(2) NUMBITS(1) [],
        /// 响应错误中断
        RE_INT_MASK OFFSET(1) NUMBITS(1) [],
        /// 卡检测中断
        CD_INT_MASK OFFSET(0) NUMBITS(1) [],
    ],
    Tmout[
        DATA_TIMEOUT OFFSET(8) NUMBITS(24) [],
        RESPONSE_TIMEOUT OFFSET(0) NUMBITS(8) [],
    ],

    CType[
        CARD0_WIDTH2 OFFSET(0) NUMBITS(16) [
            Mode1_Bit = 0,
            Mode4_Bit = 1,
        ],
        CARD0_WIDTH1 OFFSET(16) NUMBITS(16) [
            ModeNon_8_Bit = 0,
            Mode1_8_Bit = 1,
        ],
    ],
    Cmd[
        CMD_INDEX          OFFSET(0)  NUMBITS(6)  [
            SWITCH_VOLTAGE = 11,
        ],
        RESPONSE_EXPECT    OFFSET(6)  NUMBITS(1)  [],
        RESPONSE_LENGTH    OFFSET(7)  NUMBITS(1)  [],
        CHECK_RESPONSE_CRC OFFSET(8)  NUMBITS(1)  [],
        DATA_EXPECTED      OFFSET(9)  NUMBITS(1)  [],
        /// 0：读卡 1：写卡
        READ_WRITE         OFFSET(10) NUMBITS(1)  [],
        SEND_AUTO_STOP     OFFSET(12) NUMBITS(1)  [],
        WAIT_PRVDATA_COMPLETE OFFSET(13) NUMBITS(1)  [],
        STOP_ABORT_CMD     OFFSET(14) NUMBITS(1)  [],
        SEND_INITIALIZATION OFFSET(15) NUMBITS(1)  [],
        CARD_NUMBER        OFFSET(16) NUMBITS(5)  [],
        UPDATE_CLOCK_REGISTERS_ONLY OFFSET(21) NUMBITS(1)  [],
        VOLT_SWITCH        OFFSET(28) NUMBITS(1)  [],
        USE_HOLD_REG       OFFSET(29) NUMBITS(1)  [],
        START_CMD          OFFSET(30) NUMBITS(2)  [],
    ],
    Clkena[
        /// 使能
        CCLK_ENABLE OFFSET(0) NUMBITS(16) [
            Disabled = 0,
            Enabled = 1,
        ],
        /// 低功耗
        CCLK_LOW_POWER OFFSET(16) NUMBITS(16) [
            True = 0,
            False = 1,
        ],
    ],
    Status[
        FIFO_RX_WATERMARK  OFFSET(0) NUMBITS(1) [],
        FIFO_TX_WATERMARK  OFFSET(1) NUMBITS(1) [],
        FIFO_EMPTY  OFFSET(2) NUMBITS(1) [],
        FIFO_FULL  OFFSET(3) NUMBITS(1) [],
        COMMAND_FSM_STATES OFFSET(4) NUMBITS(4) [],
        DATA_3_STATUS OFFSET(8) NUMBITS(1) [],
        DATA_BUSY OFFSET(9) NUMBITS(1) [],
        DATA_STATE_MC_BUSY OFFSET(10) NUMBITS(1) [],
        RESPONSE_INDEX OFFSET(11) NUMBITS(6) [],
        FIFO_COUNT OFFSET(17) NUMBITS(13) [],
        DMA_ACK  OFFSET(30) NUMBITS(1) [],
        DMA_REQ  OFFSET(31) NUMBITS(1) [],
    ],
    UhsReg [
        VOLT_REG_0 OFFSET(0) NUMBITS(16) [
            /// 3.3V Vdd
            V3_3 = 0,
            /// 1.8V Vdd
            V1_8 = 1,
        ],
    ],
    BusMode[
        /// burst len
        PBL OFFSET(8) NUMBITS(3) [
            T1 = 0b0,
            T4 = 0b1,
            T8 = 0b10,
            T16 = 0b11,
            T32 = 0b100,
            T64 = 0b101,
            T128 = 0b110,
            T256 = 0b111,
        ],
        /// IDMAC使能
        DE OFFSET(7) NUMBITS(1) [],
        /// 固定 Burst
        ///0：SINGLE & INCR
        ///1：自动选择 SINGLE，INCR4，INCR8，或者
        ///INCR16
        FB OFFSET(1) NUMBITS(1) [],
        /// 软复位
        ///IDMA 复位内部 REG，自动清 0
        SWR OFFSET(0) NUMBITS(1) [],
    ],
    Fifoth [
        TX_WMark OFFSET(0) NUMBITS(12) [],
        RX_WMark OFFSET(16) NUMBITS(12) [],
        DMA_Multiple_Transaction_Size OFFSET(28) NUMBITS(3) [
            B1 = 0b0,
            B4 = 0b1,
            B8 = 0b10,
            B16 = 0b11,
            B32 = 0b100,
            B64 = 0b101,
            B128 = 0b110,
            B256 = 0b111,
        ],
    ],
    Cardthctl[
        /// 卡读 Threshold 使能
        CARDRDTHRE  OFFSET(0) NUMBITS(1) [],
        /// Busy 清中断
        BUSY_CLR_INT_EN OFFSET(1) NUMBITS(1) [],
        /// 写卡 Threshold 使能
        CARDWRTHREN OFFSET(2) NUMBITS(1) [],

        CARDRDTHRESHOLD_8 OFFSET(23) NUMBITS(1) [],
        CARDRDTHRESHOLD_16 OFFSET(24) NUMBITS(1) [],
        CARDRDTHRESHOLD_32 OFFSET(25) NUMBITS(1) [],
        CARDRDTHRESHOLD_64 OFFSET(26) NUMBITS(1) [],
        CARDRDTHRESHOLD_128 OFFSET(27) NUMBITS(1) [],
        CARDRDTHRESHOLD_256 OFFSET(28) NUMBITS(1) [],
    ],
    ClkSrc[
        /// 外部时钟－控制器内部设备接口模块时钟源使能。
        EXT_CLK_ENABLE OFFSET(1) NUMBITS(1) [],
        /// 分频参数，CIU f= CLK_DIV_CTRL +1，MIN=1。
        CLK_DIV_CTRL OFFSET(8) NUMBITS(7) [],
        /// 采样相位参数，相对于控制器端时钟相位点。
        CLK_SMPL_PHASE_CTRL OFFSET(16) NUMBITS(7) [],
        /// 输出相位参数，相对于控制器端时钟相位点。
        CLK_DRV_PHASE_CTRL OFFSET(24) NUMBITS(7) [],
    ],
];

impl SdRegister {
    pub fn set_fifo(&self) {
        self.fifoth.write(
            Fifoth::DMA_Multiple_Transaction_Size::B8
                + Fifoth::RX_WMark.val(0x7)
                + Fifoth::TX_WMark.val(0x100),
        );
    }

    pub fn set_card_threshold(&self) {
        self.cardthctl
            .write(Cardthctl::CARDRDTHRESHOLD_8::SET + Cardthctl::CARDRDTHRE::SET);
    }

    pub fn card_detect(&self) -> bool {
        self.card_detect.get() > 0
    }

    pub fn set_clock(&self, enable: bool) {
        self.clkena.modify(if enable {
            Clkena::CCLK_ENABLE::Enabled
        } else {
            Clkena::CCLK_ENABLE::Disabled
        });
    }

    pub fn set_power(&self, enable: bool) {
        self.pwren.set(if enable { 1 } else { 0 });
    }

    pub fn set_clock_src(&self, enable: bool) {
        self.clksrc.modify(if enable {
            ClkSrc::EXT_CLK_ENABLE::SET
        } else {
            ClkSrc::EXT_CLK_ENABLE::CLEAR
        });
    }

    pub fn update_external_clk(
        &self,
        enable: bool,
        drv_phase: u32,
        samp_phase: u32,
        clk_div: u32,
    ) -> Result {
        self.clksrc.set(0);
        self.clksrc.write(
            if enable {
                ClkSrc::EXT_CLK_ENABLE::SET
            } else {
                ClkSrc::EXT_CLK_ENABLE::CLEAR
            } + ClkSrc::CLK_DRV_PHASE_CTRL.val(drv_phase)
                + ClkSrc::CLK_SMPL_PHASE_CTRL.val(samp_phase)
                + ClkSrc::CLK_DIV_CTRL.val(clk_div),
        );

        self.wait_for(|s| s.cksts.get() > 0, true)?;

        Ok(())
    }

    pub fn disable_clock_and_update_ext_clk(&self) -> Result {
        self.set_clock(false);
        self.update_external_clk(true, 0, 0, 0x5)?;
        self.set_power(true);
        self.set_clock(true);
        self.set_clock_src(true);
        Ok(())
    }

    pub fn set_bus_witdh(&self, width: BusWitdh) {
        let val = match width {
            BusWitdh::Bit1 => CType::CARD0_WIDTH2::Mode1_Bit,
            BusWitdh::Bit4 => CType::CARD0_WIDTH2::Mode4_Bit,
            BusWitdh::Bit8 => CType::CARD0_WIDTH1::Mode1_8_Bit,
        };
        self.ctype.write(val);
    }

    pub fn send_private_cmd(&self, cmd: FieldValue<u32, Cmd::Register>, arg: u32) -> Result {
        self.wait_for(|s| s.status.is_set(Status::DATA_BUSY), false)
            .map_err(|_| Error::Busy)?;

        self.cmdarg.set(arg);

        fence(Ordering::SeqCst);

        self.cmd.write(Cmd::START_CMD.val(0b10) + cmd);

        self.wait_for(|s| s.cmd.read(Cmd::START_CMD) > 0, false)?;

        Ok(())
    }

    pub fn reset_ctrl(&self, reset_bits: FieldValue<u32, Cntrl::Register>) -> Result {
        self.cr.write(reset_bits);

        self.wait_for(|s| s.cr.get() & reset_bits.value > 0, false)?;

        /* update clock after reset */
        self.send_private_cmd(Cmd::UPDATE_CLOCK_REGISTERS_ONLY::SET, 0)?;

        /* for fifo reset, need to check if fifo empty */
        if reset_bits.value & Cntrl::FIFO_RESET::SET.value > 0 {
            self.wait_for(|s| s.status.is_set(Status::FIFO_EMPTY), true)?;
        }

        Ok(())
    }

    pub fn reset(&self, config: Config) -> Result {
        self.set_fifo();
        self.set_card_threshold();
        self.disable_clock_and_update_ext_clk()?;

        self.uhs_reg.write(UhsReg::VOLT_REG_0::V3_3);

        self.set_bus_witdh(BusWitdh::Bit1);

        let reset_bits = match config.trans_mode {
            TransMode::DMA => Cntrl::FIFO_RESET::SET + Cntrl::DMA_RESET::SET,
            TransMode::PIO => Cntrl::FIFO_RESET::SET,
        };

        self.reset_ctrl(reset_bits)
            .inspect_err(|e| error!("reset controller failed: {:?}", e))?;

        /* reset card for no-removeable media, e.g. eMMC */
        if config.non_removeable {
            self.card_reset.set(1);
        } else {
            self.card_reset.set(0);
        }

        /* clear interrupt status */
        self.int_mask.set(0);
        let mut reg_val = self.raw_ints.get();
        self.raw_ints.set(reg_val);

        self.intr_en_reg.set(0);
        reg_val = self.status_reg.get();
        self.status_reg.set(reg_val);

        /* enable card detect interrupt */
        if !config.non_removeable {
            self.int_mask.modify(IntMask::CD_INT_MASK::SET);
        }

        /* enable controller and internal DMA */
        self.cr
            .modify(Cntrl::INT_ENABLE::SET + Cntrl::USE_INTERNAL_DMAC::SET);

        /* set data and resp timeout */
        self.tmout
            .write(Tmout::DATA_TIMEOUT.val(0xffffff) + Tmout::RESPONSE_TIMEOUT.val(0xff));

        /* reset descriptors and dma */
        if let TransMode::DMA = config.trans_mode {
            self.set_descriptor(null());
            self.reset_idma();
        }

        Ok(())
    }

    fn reset_idma(&self) {
        self.bus_mode_reg.modify(BusMode::SWR::SET);
    }

    pub fn set_trans_bytes(&self, bytes: u32) {
        self.bytcnt.set(bytes);
    }

    pub fn set_block_size(&self, block_size: u32) {
        self.blksiz.set(block_size);
    }

    pub fn pio_read_data(&self, data: &mut SdifData) -> Result {
        let buf = unsafe {
            &mut *slice_from_raw_parts_mut(data.buf as *mut u32, data.datalen as usize / 4)
        };

        if data.datalen > MAX_FIFO_CNT {
            error!(
                "Fifo do not support writing more than {:#x} ({:#x})",
                MAX_FIFO_CNT, data.datalen
            );
            return Err(Error::NotSupport);
        }

        for one in buf {
            *one = self.data.get();
        }

        Ok(())
    }

    pub fn pio_write_data(&self, data: &SdifData) -> Result {
        let buf =
            unsafe { &*slice_from_raw_parts(data.buf as *mut u32, data.datalen as usize / 4) };
        self.cmd.write(Cmd::READ_WRITE::SET);
        for one in buf {
            self.data.set(*one);
        }
        Ok(())
    }

    pub fn poll_wait_busy_card<K: KFun>(&self) -> Result {
        self.wait_for_time::<K, _, _>(
            |s| {
                s.status
                    .matches_any(&[Status::DATA_BUSY::SET, Status::DATA_STATE_MC_BUSY::SET])
            },
            false,
            Duration::from_millis(200),
        )?;
        Ok(())
    }

    pub fn transfer_cmd(&self, cmd_data: &SdifCmdData) -> Result {
        let mut raw_cmd = Cmd::USE_HOLD_REG::SET;

        if cmd_data.flag.is_set(CmdDataFlag::ABORT) {
            raw_cmd += Cmd::STOP_ABORT_CMD::SET;
        }

        if cmd_data.flag.is_set(CmdDataFlag::NEED_INIT) {
            raw_cmd += Cmd::SEND_INITIALIZATION::SET;
        }

        if cmd_data.flag.is_set(SWITCH_VOLTAGE) {
            raw_cmd += Cmd::VOLT_SWITCH::SET;
        }
        /* 命令传输过程伴随数据传输 */
        if cmd_data.flag.is_set(CmdDataFlag::EXP_DATA) {
            raw_cmd += Cmd::DATA_EXPECTED::SET;
            /* 写卡 */
            if cmd_data.flag.is_set(CmdDataFlag::WRITE_DATA) {
                raw_cmd += Cmd::READ_WRITE::SET;
            }
        }
        /* 命令需要进行CRC校验 */
        if cmd_data.flag.is_set(CmdDataFlag::ADTC) {
            raw_cmd += Cmd::CHECK_RESPONSE_CRC::SET;
        }
        /* 命令需要响应回复 */
        if cmd_data.flag.is_set(CmdDataFlag::EXP_RESP) {
            raw_cmd += Cmd::RESPONSE_EXPECT::SET;
            /* 命令需要136字节长回复 */
            if cmd_data.flag.is_set(CmdDataFlag::EXP_LONG_RESP) {
                raw_cmd += Cmd::RESPONSE_LENGTH::SET;
            }
        }

        raw_cmd += Cmd::CMD_INDEX.val(cmd_data.cmdidx);

        debug!("============[CMD-{}] begin ============", cmd_data.cmdidx,);
        debug!("    cmd: {:#x}", raw_cmd.value);
        debug!("    arg: {:#x}", cmd_data.cmdarg);
        /* enable related interrupt */
        self.int_mask.modify(IntMask::CMD_INT_MASK::SET);

        self.send_private_cmd(raw_cmd, cmd_data.cmdarg)?;

        debug!("cmd send done ...");

        Ok(())
    }

    pub fn reset_fifo_and_not_use_dma(&self) -> Result {
        self.cr.modify(Cntrl::USE_INTERNAL_DMAC::CLEAR);
        self.reset_ctrl(Cntrl::FIFO_RESET::SET)?;

        self.bus_mode_reg.modify(BusMode::DE::CLEAR);
        Ok(())
    }

    fn set_descriptor(&self, descriptor: *const u8) {
        let ptr = descriptor as usize;
        #[cfg(target_arch = "aarch64")]
        {
            self.desc_list_star_reg_u.set((ptr >> 32) as u32);
            self.desc_list_star_reg_l.set((ptr & 0xFFFFFFFF) as u32);
        }
        #[cfg(target_arch = "arm")]
        {
            self.desc_list_star_reg_u.set(0);
            self.desc_list_star_reg_l.set(ptr as u32);
        }
    }
    fn wait_for_time<K: KFun, T: Eq, F>(&self, f: F, want: T, duration: Duration) -> Result
    where
        F: Fn(&SdRegister) -> T,
    {
        let count = duration.as_micros() / 10;

        for _ in 0..count {
            if f(self).eq(&want) {
                return Ok(());
            }
            K::sleep(Duration::from_micros(10));
        }

        Err(Error::Timeout)
    }
    fn wait_for<T: Eq, F>(&self, f: F, want: T) -> Result
    where
        F: Fn(&SdRegister) -> T,
    {
        for _ in 0..TIMEOUT {
            if f(self).eq(&want) {
                return Ok(());
            }
            spin_loop();
        }

        Err(Error::Timeout)
    }
}

#[allow(unused)]
#[derive(Debug, Clone, Copy)]
pub enum BusWitdh {
    Bit1,
    Bit4,
    Bit8,
}

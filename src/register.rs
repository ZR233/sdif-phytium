use tock_registers::{
    interfaces::{ReadWriteable, Readable, Writeable},
    register_bitfields, register_structs,
    registers::*,
};

use crate::err::{Error, Result};

const TIMEOUT: usize = 50000;

register_structs! {
    pub SdRegister {
       // Control register: read-write
        (0x000 => cr: ReadWrite<u32>),
        // Power enable register: read-write
        (0x004 => pwren: ReadWrite<u32>),
        // Clock divider register: read-write
        (0x008 => clkdiv: ReadWrite<u32>),
        (0x00C => _rsv),
        // Card clock enable register: read-write
        (0x010 => clkena: ReadWrite<u32, Clkena::Register>),
        // Timeout register: read-write
        (0x014 => tmout: ReadWrite<u32>),
        // Card type register: read-write
        (0x018 => ctype: ReadWrite<u32, CType::Register>),
        // Block size register: read-write
        (0x01C => blksiz: ReadWrite<u32>),
        // Byte count register: read-write
        (0x020 => bytcnt: ReadWrite<u32>),
        // Interrupt mask register: read-write
        (0x024 => int_mask: ReadWrite<u32>),
        // Command argument register: read-write
        (0x028 => cmdarg: ReadWrite<u32>),
        // Command register: read-write
        (0x02C => cmd: ReadWrite<u32>),
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
        (0x044 => raw_ints: ReadOnly<u32>),
        // Controller status register: read-only
        (0x048 => status: ReadOnly<u32>),
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
        (0x080 => bus_mode_reg: ReadWrite<u32>),
        (0x084 => _rsv2),
        // Descriptor list start address register low: read-write
        (0x088 => desc_list_star_reg_l: ReadWrite<u32>),
        // Descriptor list start address register high: read-write
        (0x08C => desc_list_star_reg_u: ReadWrite<u32>),
        // Internal DMAC status register: read-only
        (0x090 => status_reg: ReadOnly<u32>),
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
    UhsReg [
        VOLT_REG_0 OFFSET(0) NUMBITS(16) [
            /// 3.3V Vdd
            V3_3 = 0,
            /// 1.8V Vdd
            V1_8 = 1,
        ],
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

        for _ in 0..TIMEOUT {
            if self.clksrc.get() == 0 {
                return Ok(());
            }
        }

        Err(Error::Timeout)
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

    pub fn reset(&self) -> Result {
        self.set_fifo();
        self.set_card_threshold();
        self.disable_clock_and_update_ext_clk()?;

        self.uhs_reg.write(UhsReg::VOLT_REG_0::V3_3);

        self.set_bus_witdh(BusWitdh::Bit1);

        Ok(())
    }
}

#[allow(unused)]
#[derive(Debug, Clone, Copy)]
pub enum BusWitdh {
    Bit1,
    Bit4,
    Bit8,
}

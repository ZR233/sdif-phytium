use tock_registers::{register_structs, registers::*};

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
        (0x010 => clkena: ReadWrite<u32>),
        // Timeout register: read-write
        (0x014 => tmout: ReadWrite<u32>),
        // Card type register: read-write
        (0x018 => ctype: ReadWrite<u32>),
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
        (0x04C => fifoth: ReadWrite<u32>),
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
        (0x074 => uhs_reg: ReadWrite<u32>),
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
        (0x100 => cardthctl: ReadWrite<u32>),
        (0x104 => _rsv4),
        // UHS register extension: read-write
        (0x108 => clksrc: ReadWrite<u32>),
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

use tock_registers::{register_bitfields, registers::ReadWrite};

#[repr(C)]
pub struct SdifData {
    pub buf: *mut u8,   // 虚拟地址缓冲区
    pub buf_dma: usize, // 物理地址用于DMA
    pub blksz: u32,     // 卡块大小
    pub blkcnt: u32,    // 传输中的块数
    pub datalen: u32,   // 传输中的字节数
}

#[repr(C)]
pub struct SdifCmdData {
    pub cmdidx: u32,                                 // 命令索引
    pub cmdarg: u32,                                 // 命令参数
    pub response: [u32; 4],                          // 命令响应缓冲区
    pub flag: ReadWrite<u32, CmdDataFlag::Register>, // 命令标志
    pub data_p: *mut SdifData,                       // SDIF传输数据
    pub success: bool,                               // TRUE: 命令和数据传输成功
}

register_bitfields! [
    u32,
    pub CmdDataFlag[
        NEED_INIT OFFSET(0) NUMBITS(1) [],
        EXP_RESP     OFFSET(1) NUMBITS(1) [],
        EXP_LONG_RESP     OFFSET(3) NUMBITS(1) [],
        NEED_RESP_CRC     OFFSET(4) NUMBITS(1) [],
        EXP_DATA OFFSET(5) NUMBITS(1) [], // 需要传输数据
        WRITE_DATA OFFSET(6) NUMBITS(1) [], // 需要将数据写入卡
        READ_DATA OFFSET(7) NUMBITS(1) [],// 需要从卡读取数据
        NEED_AUTO_STOP OFFSET(8) NUMBITS(1) [],// 需要在命令后自动停止
        ADTC  OFFSET(5) NUMBITS(9) [], // 需要ADTC
        SWITCH_VOLTAGE  OFFSET(10) NUMBITS(1) [],// 需要切换电压
        ABORT  OFFSET(5) NUMBITS(11) [],// 需要中止
    ],
];

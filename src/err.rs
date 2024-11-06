pub type Result<T = ()> = core::result::Result<T, Error>;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    Timeout,
    NotInit,
    ShortBuf,
    NotSupport,
    InvalidState,
    TransTimeout,
    CmdTimeout,
    NoCard,
    Busy,
    DmaBufUnalign,
    InvalidTiming,
}

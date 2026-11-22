use core::convert::TryFrom;
use num_enum::{IntoPrimitive, TryFromPrimitive};

const PREFIX: u16 = 0b01 << 14;
const MASK_BIT: u16 = 0x3FFF;

#[derive(Debug, Clone, Copy, PartialEq, Eq, IntoPrimitive, TryFromPrimitive)]
#[repr(u16)]
pub enum DownlinkInfo {
    Diasable = 0x0001,
    Enable = 0x0002,
    MeasureStart = 0x0010,
    MeasureStop = 0x0011,
    Reset = 0x3FFF,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParseError {
    InvalidPrefix, // 上位2bitが期待値と異なる
    UnknownValue,  // 下位14bitが未定義の値
}

impl From<DownlinkInfo> for [u8;2]{
    fn from(info: DownlinkInfo)-> Self{
        let raw = PREFIX | (info as u16);
        raw.to_le_bytes()
    }
}

impl  TryFrom<[u8;2]> for DownlinkInfo{
    type Error = ();
    fn try_from(value: [u8;2]) -> Result<Self, Self::Error> {
        let raw = u16::from_le_bytes(value);
        if (raw & !MASK_BIT) != PREFIX {
            return Err(());
        }
        Self::try_from(raw & MASK_BIT).map_err(|_|())
    }
}
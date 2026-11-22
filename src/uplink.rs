use crate::common::Aligned16;
use crc::{CRC_16_MODBUS, Crc};
use zerocopy::{FromBytes, Immutable, IntoBytes, KnownLayout};

pub const UPLINK_SIZE: usize = 20; //回転数or推力(4*4B)+その他情報(2B)+CRC-16(2B)
pub const PAYLOAD_SIZE: usize = 18;
const _: () = assert!(PAYLOAD_SIZE + 2 == UPLINK_SIZE);

#[derive(FromBytes, KnownLayout, Immutable, IntoBytes)]
#[repr(C, align(4))]
pub struct UplinkBytes(pub [u8; UPLINK_SIZE]);

impl UplinkBytes {
    pub const fn new() -> Self {
        Self([0; UPLINK_SIZE])
    }
}

const CRITICAL_BIT: u8 = 1 << 7;
const INFO_BIT: u8 = 1 << 6;
const CRC16: Crc<u16> = Crc::<u16>::new(&CRC_16_MODBUS);

#[derive(FromBytes, IntoBytes, KnownLayout, Immutable)]
#[repr(C, align(4))]
pub struct UplinkFrame {
    thrust_estim: Aligned16,
    info: [u8; 2],
    trailer: [u8; 2],
}

pub enum UplinkPayload<'a> {
    Normal(UplinkPayloadNormal<'a>),
    Critical(UplinkPayloadCritical<'a>),
}

pub struct UplinkPayloadNormal<'a> {
    thrust_estim: &'a [f32; 4],
    info: Option<[u8; 2]>,
}

impl<'a> UplinkPayloadNormal<'a> {
    pub fn get_thrust_estim(&self) -> &'a [f32; 4] {
        self.thrust_estim
    }
    pub fn get_info(&self) -> Option<[u8; 2]> {
        self.info
    }
    pub fn pack(thrust_estim: &'a [f32; 4], info: Option<[u8; 2]>) -> Self {
        Self { thrust_estim, info }
    }
}

pub struct UplinkPayloadCritical<'a> {
    critical_info: &'a [u8; PAYLOAD_SIZE],
}

impl<'a> UplinkPayloadCritical<'a> {
    pub fn pack(critical_info: &'a [u8; PAYLOAD_SIZE]) -> Self {
        Self { critical_info }
    }

    pub fn get_critical_info(&self) -> &'a [u8; PAYLOAD_SIZE] {
        self.critical_info
    }
}

impl UplinkBytes {
    pub fn decode<'a>(&'a self) -> Option<UplinkPayload<'a>> {
        let parsed_frame: &UplinkFrame = zerocopy::transmute_ref!(self);
        let crc = u16::from_le_bytes(parsed_frame.trailer);
        if crc != CRC16.checksum(&self.0[..PAYLOAD_SIZE]) {
            return None;
        } else {
            if parsed_frame.info[0] & CRITICAL_BIT != 0 {
                return Some(UplinkPayload::Critical(UplinkPayloadCritical {
                    critical_info: parsed_frame
                        .as_bytes()
                        .first_chunk::<PAYLOAD_SIZE>()
                        .unwrap(),
                }));
            } else {
                if parsed_frame.info[0] & INFO_BIT != 0 {
                    Some(UplinkPayload::Normal(UplinkPayloadNormal {
                        thrust_estim: zerocopy::transmute_ref!(&parsed_frame.thrust_estim),
                        info: Some(parsed_frame.info),
                    }))
                } else {
                    Some(UplinkPayload::Normal(UplinkPayloadNormal {
                        thrust_estim: zerocopy::transmute_ref!(&parsed_frame.thrust_estim),
                        info: None,
                    }))
                }
            }
        }
    }
    pub fn encode(&mut self, payload: UplinkPayload) {
        let frame: &mut UplinkFrame = zerocopy::transmute_mut!(self);

        match payload {
            UplinkPayload::Normal(payload) => {
                frame.thrust_estim = zerocopy::transmute!(*payload.thrust_estim);
                frame.info = match payload.info {
                    Some(mut info) => {
                        info[0] = (info[0] | INFO_BIT) & !CRITICAL_BIT;
                        info
                    }
                    None => [0; 2],
                };
            }
            UplinkPayload::Critical(message) => {
                // 先頭18バイト(thrust_estim + info)に直接コピー
                frame.as_mut_bytes()[..PAYLOAD_SIZE].copy_from_slice(message.critical_info);
                frame.info[0] |= CRITICAL_BIT;
            }
        }

        let crc = CRC16.checksum(&frame.as_bytes()[..PAYLOAD_SIZE]);
        frame.trailer = crc.to_le_bytes();
    }
}

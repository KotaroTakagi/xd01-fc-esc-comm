use crate::common::Aligned16;
pub mod info;
pub use info::DownlinkInfo;
use crc::{CRC_16_MODBUS, Crc};
use zerocopy::{FromBytes, Immutable, IntoBytes, KnownLayout};

pub const DOWNLINK_SIZE: usize = 36; //指令値(4*4B)+流入速度(4*4B)+その他情報(2B)+CRC-16(2B)
pub const PAYLOAD_SIZE: usize = 34;
const _: () = assert!(PAYLOAD_SIZE + 2 == DOWNLINK_SIZE);

#[derive(FromBytes, KnownLayout, Immutable, IntoBytes)]
#[repr(C, align(4))]
pub struct DownlinkBytes(pub [u8; DOWNLINK_SIZE]);

impl DownlinkBytes {
    pub const fn new() -> Self {
        Self([0; DOWNLINK_SIZE])
    }
}

const CRITICAL_BIT: u8 = 1 << 7;
const INFO_BIT: u8 = 1 << 6;
const CRC16: Crc<u16> = Crc::<u16>::new(&CRC_16_MODBUS);

#[derive(FromBytes, IntoBytes, KnownLayout, Immutable)]
#[repr(C, align(4))]
pub struct DownlinkFrame {
    thrust_ref: Aligned16,
    inlet_velocity: Aligned16,
    info: [u8; 2],
    trailer: [u8; 2],
}

pub enum DownlinkPayload<'a> {
    Normal(DownlinkPayloadNormal<'a>),
    Critical(DownlinkPayloadCritical<'a>),
}

pub struct DownlinkPayloadNormal<'a> {
    thrust_ref: &'a [f32; 4],
    inlet_velocity: &'a [f32; 4],
    info: Option<DownlinkInfo>,
}

impl<'a> DownlinkPayloadNormal<'a> {
    pub fn get_thrust_ref_ref(&self) -> &'a [f32; 4] {
        self.thrust_ref
    }
    pub fn get_inlet_velocity(&self) -> &'a [f32; 4] {
        self.inlet_velocity
    }
    pub fn get_info(&self) -> Option<DownlinkInfo> {
        self.info
    }
    pub fn pack(
        thrust_ref: &'a [f32; 4],
        inlet_velocity: &'a [f32; 4],
        info: Option<DownlinkInfo>,
    ) -> Self {
        Self {
            thrust_ref,
            inlet_velocity,
            info,
        }
    }
}

pub struct DownlinkPayloadCritical<'a> {
    critical_info: &'a [u8; PAYLOAD_SIZE],
}

pub enum DownlinkErr{
    CRCErr,
    InfoParseErr,
}

impl<'a> DownlinkPayloadCritical<'a> {
    pub fn pac(critical_info: &'a [u8; PAYLOAD_SIZE]) -> Self {
        Self { critical_info }
    }

    pub fn get_critical_info(&self) -> &'a [u8; PAYLOAD_SIZE] {
        self.critical_info
    }
}

impl DownlinkBytes {
    pub fn decode<'a>(&'a self) -> Result<DownlinkPayload<'a>,DownlinkErr> {
        let parsed_frame: &DownlinkFrame = zerocopy::transmute_ref!(self);
        let crc = u16::from_le_bytes(parsed_frame.trailer);
        if crc != CRC16.checksum(&self.0[..PAYLOAD_SIZE]) {
            return Err(DownlinkErr::CRCErr);
        } else {
            if parsed_frame.info[0] & CRITICAL_BIT != 0 {
                return Ok(DownlinkPayload::Critical(DownlinkPayloadCritical {
                    critical_info: parsed_frame.as_bytes().first_chunk::<34>().unwrap(),
                }));
            } else {
                if parsed_frame.info[0] & INFO_BIT != 0 {
                    match DownlinkInfo::try_from(parsed_frame.info){
                        Ok(info) => Ok(DownlinkPayload::Normal(DownlinkPayloadNormal {
                        thrust_ref: zerocopy::transmute_ref!(&parsed_frame.thrust_ref),
                        inlet_velocity: zerocopy::transmute_ref!(&parsed_frame.inlet_velocity),
                        info: Some(info),
                    })),
                    Err(_) => Err(DownlinkErr::InfoParseErr)
                    }
                } else {
                    Ok(DownlinkPayload::Normal(DownlinkPayloadNormal {
                        thrust_ref: zerocopy::transmute_ref!(&parsed_frame.thrust_ref),
                        inlet_velocity: zerocopy::transmute_ref!(&parsed_frame.inlet_velocity),
                        info: None,
                    }))
                }
            }
        }
    }

    pub fn encode(&mut self, payload: DownlinkPayload) {
        let frame: &mut DownlinkFrame = zerocopy::transmute_mut!(self);

        match payload {
            DownlinkPayload::Normal(payload) => {
                frame.thrust_ref = zerocopy::transmute!(*payload.thrust_ref);
                frame.inlet_velocity = zerocopy::transmute!(*payload.inlet_velocity);
                frame.info = match payload.info {
                    Some(info) => {
                        let mut info:[u8;2] = info.into(); 
                        info[0] = (info[0] | INFO_BIT) & !CRITICAL_BIT;
                        info
                    }
                    None => [0; 2],
                };
            }
            DownlinkPayload::Critical(message) => {
                frame.as_mut_bytes()[..PAYLOAD_SIZE].copy_from_slice(message.critical_info);
                frame.info[0] |= CRITICAL_BIT;
            }
        }

        let crc = CRC16.checksum(&frame.as_bytes()[..PAYLOAD_SIZE]);
        frame.trailer = crc.to_le_bytes();
    }
}

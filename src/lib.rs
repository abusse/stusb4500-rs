#![no_std]

extern crate bitflags;
extern crate byteorder;
extern crate embedded_hal as hal;
extern crate std;

use std::convert::TryInto;

use byteorder::{ByteOrder, LittleEndian};
use hal::blocking::i2c;

pub mod pdo;
pub mod rdo;
pub mod registers;

use pdo::*;
use rdo::*;
use registers::*;

pub const STUSB4500_ADDR: u8 = 0x28;

/// Address enum for STUSB4500
pub enum Address {
    /// Default address with all address pins tied low
    Default,
    /// Address determined by A1 and A0 pins. True = tied high, low = tied low.
    Strap(bool, bool),
    /// Custom address from config file etc
    Custom(u8),
}

impl Address {
    pub(crate) fn addr(&self) -> u8 {
        match self {
            Address::Default => STUSB4500_ADDR,
            Address::Strap(a1, a0) => STUSB4500_ADDR | (*a1 as u8) << 1 | (*a0 as u8) << 0,
            Address::Custom(addr) => *addr,
        }
    }
}

impl Default for Address {
    fn default() -> Self {
        Address::Default
    }
}

#[derive(Debug)]
pub enum Error<I2C> {
    I2CError(I2C),
    InvalidPdo,
    OutaRangePdo,
}

pub enum PdoChannel {
    PDO1,
    PDO2,
    PDO3,
}

pub struct STUSB4500<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C, E> STUSB4500<I2C>
where
    I2C: i2c::Write<Error = E> + i2c::Read<Error = E>,
{
    pub fn new(i2c: I2C, address: Address) -> Self {
        STUSB4500 {
            i2c,
            address: address.addr(),
        }
    }

    /// Read all interrupt registers to clear them
    pub fn clear_interrupts(&mut self) -> Result<(), Error<E>> {
        // Read all interrupt registers
        let mut _buf = [0x00; 10];
        self.i2c
            .write(self.address, &[Register::PortStatus0 as u8])
            .map_err(|err| Error::I2CError(err))?;
        self.i2c
            .read(self.address, &mut _buf)
            .map_err(|err| Error::I2CError(err))
    }

    /// Set interrupt mask
    pub fn set_alerts_mask(&mut self, alerts: AlertMask) -> Result<(), Error<E>> {
        self.write(Register::AlertStatus1Mask, alerts.bits())
    }

    /// Get active interrupt flags
    pub fn get_alerts(&mut self) -> Result<Alert, Error<E>> {
        Ok(Alert::from_masked_bits(self.read(Register::AlertStatus1)?))
    }

    /// Perform a soft reset
    /// Triggers re-negotiation of PDO's.
    pub fn soft_reset(&mut self) -> Result<(), Error<E>> {
        self.write(Register::TXHeaderL, 0x0D)?;
        self.write(Register::PDCommandCtrl, 0x26)?;
        Ok(())
    }

    pub fn set_pdo(&mut self, pdo: PdoChannel, data: &Pdo) -> Result<(), Error<E>> {
        if let Pdo::Fixed { .. } = data {
            self.write_word(
                match pdo {
                    PdoChannel::PDO1 => Register::DPMSNKPDO1,
                    PdoChannel::PDO2 => Register::DPMSNKPDO2,
                    PdoChannel::PDO3 => Register::DPMSNKPDO3,
                },
                data.bits(),
            )
        } else {
            // Can only advertise fixed PDOs
            Err(Error::InvalidPdo)
        }
    }

    pub fn get_pdo(&mut self, pdo: PdoChannel) -> Result<Pdo, Error<E>> {
        Pdo::from_bits(self.read_word(match pdo {
            PdoChannel::PDO1 => Register::DPMSNKPDO1,
            PdoChannel::PDO2 => Register::DPMSNKPDO2,
            PdoChannel::PDO3 => Register::DPMSNKPDO3,
        })?)
        .ok_or(Error::InvalidPdo)
    }

    pub fn get_current_rdo(&mut self) -> Result<Rdo, Error<E>> {
        Ok(Rdo(self.read_word(Register::RDORegStatus)?))
    }

    pub fn set_num_pdo(&mut self, num: u8) -> Result<(), Error<E>> {
        match num {
            1..=3 => self.write(Register::DPMPDONumb, num),
            _ => Err(Error::OutaRangePdo),
        }
    }

    pub fn get_nvm(&mut self) -> Result<[u64; 5], Error<E>> {
        let mut buf = [0x00; 5];
        self.set_nvm_lock(false)?;
        for x in 0..5 {
            match self.read_nvm_sector(x as u8) {
                Ok(value) => {
                    buf[x] = value;
                }
                Err(err) => return Err(err),
            }
        }
        self.set_nvm_lock(true)?;

        Ok(buf)
    }

    pub fn get_nvm_bytes(&mut self) -> Result<[u8; 40], Error<E>> {
        let mut buf = [0x00; 40];

        let data = self.get_nvm()?;
        for x in 0..5 {
            buf[x * 8..(x * 8) + 8].clone_from_slice(&(data[x].to_le_bytes()));
        }

        Ok(buf)
    }

    pub fn write_nvm(&mut self, data: [u64; 5]) -> Result<(), Error<E>> {
        self.set_nvm_lock(false)?;
        self.delete_nvm()?;
        for x in 0..5 {
            self.write_nvm_sector(x as u8, data[x])?;
        }
        self.set_nvm_lock(true)?;

        Ok(())
    }

    pub fn write_nvm_bytes(&mut self, data: [u8; 40]) -> Result<(), Error<E>> {
        let mut buf: [u64; 5] = [0; 5];

        for x in 0..5 {
            buf[x] = u64::from_le_bytes(data[x * 8..(x * 8) + 8].try_into().unwrap());
        }

        self.write_nvm(buf)
    }

    // *****************************************************************
    // NVM helper functions

    fn nvm_wait(&mut self) -> Result<(), Error<E>> {
        loop {
            match self.read(Register::CTRL0) {
                Ok(value) => {
                    if value & CTRL0CmdMask::REQ.bits() == 0x00 {
                        return Ok(());
                    }
                }
                Err(err) => return Err(err),
            }
        }
    }

    fn set_nvm_lock(&mut self, lock: bool) -> Result<(), Error<E>> {
        if lock {
            self.write(Register::Password, 0x00)?;
        } else {
            self.write(Register::Password, 0x47)?;
        }
        self.nvm_wait()
    }

    fn delete_nvm(&mut self) -> Result<(), Error<E>> {
        self.write(Register::CTRL1, 0xFA)?;
        self.write(Register::CTRL0, CTRL0CmdMask::_Default.bits())?;
        self.nvm_wait()?;
        self.write(Register::CTRL1, NVMCmd::SoftProgSector as u8)?;
        self.write(Register::CTRL0, CTRL0CmdMask::_Default.bits())?;
        self.nvm_wait()?;
        self.write(Register::CTRL1, NVMCmd::EraseSector as u8)?;
        self.write(Register::CTRL0, CTRL0CmdMask::_Default.bits())?;
        self.nvm_wait()?;

        Ok(())
    }

    fn write_nvm_sector(&mut self, sector: u8, data: u64) -> Result<(), Error<E>> {
        self.write_double_word(Register::RWBuffer, data)?;
        self.write(Register::CTRL1, NVMCmd::WritePL as u8)?;
        self.write(Register::CTRL0, CTRL0CmdMask::_Default.bits())?;
        self.nvm_wait()?;
        self.write(Register::CTRL1, NVMCmd::ProgSector as u8)?;
        self.write(
            Register::CTRL0,
            CTRL0CmdMask::_Default.bits() | (sector & CTRL0CmdMask::SECT.bits()),
        )?;
        self.nvm_wait()?;

        Ok(())
    }

    fn read_nvm_sector(&mut self, sector: u8) -> Result<u64, Error<E>> {
        self.write(Register::CTRL1, NVMCmd::Read as u8)?;
        self.write(
            Register::CTRL0,
            CTRL0CmdMask::_Default.bits() | (sector & CTRL0CmdMask::SECT.bits()),
        )?;
        self.nvm_wait()?;
        self.read_double_word(Register::RWBuffer)
    }

    // *****************************************************************
    // Raw access functions

    /// Write a byte register
    pub(crate) fn write(&mut self, register: Register, value: u8) -> Result<(), Error<E>> {
        let buf = [register as u8, value];
        self.i2c
            .write(self.address, &buf)
            .map_err(|err| Error::I2CError(err))
    }

    /// Write a word register
    pub(crate) fn write_word(&mut self, register: Register, word: u32) -> Result<(), Error<E>> {
        let mut buf = [0x00; 5];
        buf[0] = register as u8;
        LittleEndian::write_u32(&mut buf[1..], word);
        self.i2c
            .write(self.address, &buf)
            .map_err(|err| Error::I2CError(err))
    }

    /// Write a double word register
    pub(crate) fn write_double_word(
        &mut self,
        register: Register,
        word: u64,
    ) -> Result<(), Error<E>> {
        let mut buf = [0x00; 9];
        buf[0] = register as u8;
        LittleEndian::write_u64(&mut buf[1..], word);
        self.i2c
            .write(self.address, &buf)
            .map_err(|err| Error::I2CError(err))
    }

    /// Read a byte register
    pub(crate) fn read(&mut self, register: Register) -> Result<u8, Error<E>> {
        let mut buf = [0x00; 1];
        self.i2c
            .write(self.address, &[register as u8])
            .map_err(|err| Error::I2CError(err))?;
        self.i2c
            .read(self.address, &mut buf)
            .map_err(|err| Error::I2CError(err))?;
        Ok(buf[0])
    }

    /// Read a word register
    pub(crate) fn read_word(&mut self, register: Register) -> Result<u32, Error<E>> {
        let mut buf = [0x00; 4];
        self.i2c
            .write(self.address, &[register as u8])
            .map_err(|err| Error::I2CError(err))?;
        self.i2c
            .read(self.address, &mut buf)
            .map_err(|err| Error::I2CError(err))?;
        Ok(LittleEndian::read_u32(&buf))
    }

    /// Read a double word register
    pub(crate) fn read_double_word(&mut self, register: Register) -> Result<u64, Error<E>> {
        let mut buf = [0x00; 8];
        self.i2c
            .write(self.address, &[register as u8])
            .map_err(|err| Error::I2CError(err))?;
        self.i2c
            .read(self.address, &mut buf)
            .map_err(|err| Error::I2CError(err))?;
        Ok(LittleEndian::read_u64(&buf))
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}

use clap::{App, AppSettings, Arg, SubCommand};
use linux_embedded_hal::I2cdev;
use std::convert::TryInto;
use std::process;
use stusb4500::*;

const FACTORY_NVM: &str = "0xF0 0x00 0xB0 0xAA 0x00 0x45 0x00 0x00
                           0x10 0x40 0x9C 0x1C 0xF0 0x01 0x00 0xDF
                           0x02 0x40 0x0F 0x00 0x32 0x00 0xFC 0xF1
                           0x00 0x19 0x54 0xAF 0xF5 0x35 0x5F 0x00
                           0x00 0x2D 0x2C 0x21 0x43 0x00 0x40 0xFB";

fn main() {
    let matches = App::new("STUSB4500 NVM Programmer")
        .version("0.2.0")
        .about("Example program that uses the stusb400 library to read and write the chips NVM.")
        .arg(
            Arg::with_name("i2c-dev")
                .help("Dev node of I2C bus")
                .required(true)
                .index(1),
        )
        .setting(AppSettings::SubcommandRequiredElseHelp)
        .subcommand(SubCommand::with_name("read").about("Prints the NVM content"))
        .subcommand(
            SubCommand::with_name("write")
                .about("Writes the NVM content")
                .arg(
                    Arg::with_name("DATA")
                        .help("Data to be written as hex string")
                        .required_unless("default")
                        .required(true)
                        .index(1),
                )
                .arg(
                    Arg::with_name("default")
                        .help("Writes the factory default")
                        .long("default")
                        .short("d")
                        .takes_value(false),
                ),
        )
        .get_matches();

    let dev = match I2cdev::new(matches.value_of("i2c-dev").unwrap()) {
        Ok(dev) => dev,
        Err(error) => {
            eprintln!("Cannot access I2C bus: {}", error);
            process::exit(1);
        }
    };

    let stusb = STUSB4500::new(dev, Address::Default);

    if let Some(_) = matches.subcommand_matches("read") {
        read(stusb);
    } else if let Some(matches) = matches.subcommand_matches("write") {
        if matches.is_present("default") {
            write(stusb, FACTORY_NVM);
        } else {
            write(stusb, matches.value_of("DATA").unwrap());
        }
    }
}

fn read(mut stusb: STUSB4500<I2cdev>) {
    let nvm_content = match stusb.get_nvm_bytes() {
        Ok(value) => value,
        Err(error) => {
            eprintln!("Reading USB4500 NVM failed: {:?}", error);
            process::exit(1);
        }
    };

    for bank in nvm_content.iter() {
        for value in bank.to_le_bytes().iter() {
            print!("{:#04X?} ", value);
        }
        println!("");
    }
}

fn write(mut stusb: STUSB4500<I2cdev>, data: &str) {
    let data = match hex::decode(
        data.replace("0x", "")
            .chars()
            .filter(|c| !c.is_whitespace())
            .collect::<String>(),
    ) {
        Ok(value) => value,
        Err(error) => {
            eprintln!("Data invalid: {:?}", error);
            process::exit(1);
        }
    };

    let buf: [u8; 40] = match data.try_into() {
        Ok(data) => data,
        Err(error) => {
            eprintln!("Data invalid: {} Bytes instead of 40", error.len());
            process::exit(1);
        }
    };

    match stusb.write_nvm_bytes(buf) {
        Ok(_) => (),
        Err(error) => {
            eprintln!("Failed to write data to NVM: {:?}", error);
            process::exit(1);
        }
    }
}

cargo build --release
cargo objcopy --bin fake_rotor --target thumbv7m-none-eabi --release -- -O binary fake_rotor.bin
rem st-flash erase
rem st-flash write fake_rotor.bin 0x8000000
rem cargo embed --release --chip STM32F103C8
cargo embed --release --chip STM32F103C8



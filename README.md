# This is the first rust build release of rockchip npu, cpu, memory, soc thermal usage meter

## Build instruction
### Install the arm arm gnu gcc in ubuntu or debian based system
```
sudo apt install gcc-arm-linux-gnueabihf
```
### Install rust
Install full rust toolchain from this website
https://www.rust-lang.org/tools/install

### Clone this repository
```
git clone https://github.com/Biswas005/rknuptop.git
cd rknuptop
```

### Add rust toolchain arm muslc toolchain
```
rustup target add armv7-unknown-linux-musleabihf
```

### Build 
```
cargo build --target=armv7-unknown-linux-musleabihf --release
```
This command already optimizes your code

### Binary location
#### if inside the rknuptop directory
```
cd target/armv7-unknown-linux-musleabihf/release
```
#### if not inside the rknuptop
```
cd ~/{PATH-TO-rknuptop}/rknuptop/target/armv7-unknown-linux-musleabihf/release
```

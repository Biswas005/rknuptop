# This is the first rust build release of rockchip npu, cpu, memory, soc thermal usage meter( It is build upon arm 32 bit tested on ev1106g3)

## Build instruction
### Install the arm arm gnu gcc in ubuntu or debian based system
```
sudo apt install gcc-arm-linux-gnueabihf
```
### or for fedora
```
sudo dnf copr enable lantw44/arm-linux-gnueabihf-toolchain
sudo dnf install arm-linux-gnueabihf-{binutils,gcc,glibc}
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
#### If inside the rknuptop directory
```
cd target/armv7-unknown-linux-musleabihf/release
```
#### If not inside the rknuptop
```
cd ~/{PATH-TO-rknuptop}/rknuptop/target/armv7-unknown-linux-musleabihf/release
```

## Working picture
![Alt text](rknputop.png)

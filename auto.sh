for TARGET in \
  armv7-unknown-linux-gnueabihf \
  armv7-unknown-linux-musleabihf \
  aarch64-unknown-linux-gnu \
  aarch64-unknown-linux-musl
do
  cargo build --target=$TARGET --release
done

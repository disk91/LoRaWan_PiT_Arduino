# Arduino / LoRaWan PiT Info
This project has been made to report the PIT Info (Electrical Consumption like for linky but much more) over LoRaWan.
The project is basic only works for "Base" Information corresponding to the overall energy counter. It's going to be hard to extend it on the tiny hardware used for running it.

## Hardware
- Arduino LoRaRadio Node board
- [PiTIUnfo board from Charles (ch2i)](https://www.tindie.com/products/hallard/pitinfo/)
- some wire
- 2x 1/2 AA 3,6V batteries

## Mounting
- The Pit Board is connected with TX to D4, VCC to D3, GND to GND

## Data format
Base is the global energy counter Value
Delta is the difference between two tranmissions (keep in mind that some packets will be lost)

| byte | means |
|----|-----|
| 0 | base & 0xFF000000|
| 1 | base & 0x00FF0000|
| 2 | base & 0x0000FF00|
| 3 | base & 0x000000FF|
| 4 | delta & 0x00FF0000|
| 5 | delta & 0x0000FF00|
| 6 | delta & 0x000000FF|

## Helium decoder
```JS
function Decoder(bytes, port, uplink_info) {
  decoded = {
     base : (bytes[0] << 24) + (bytes[1] << 16) + (bytes[2] << 8) + bytes[3],
     delta : (bytes[4] << 16) + (bytes[5] << 8) + bytes[6] 
  };

  return decoded;
}
```
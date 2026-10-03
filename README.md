# xmip-core-transport-iso-tp

ISO-TP transport: ISO 15765-2 over CAN — single, first, consecutive and flow-control frames segment a payload of up to 4095 bytes across the bus and reassemble it, block size and separation time from the receiver. A technology of [xmip-core-transport](https://github.com/IlleNilsson/xmip-core-transport).

`bus_of` reads the bus back out of an origin this technology wrote, `isotp://<bus>/0x<id>`, for UDS and OBD-II, which name the bus in their own origins; until 2026-09-28 each of them cut it out of the origin itself.

## Acknowledgement

Acceptance is at-most-once here. ISO 15765-2 acknowledges no message it
completed: flow control paces the sender before the data, and nothing answers
the last frame, so nobody is left to tell how the receive cycle ended. Each
message arrives whole. UDS, riding on this crate, answers its requests itself.

## Toolchain

`rust-toolchain.toml` pins the toolchain for the whole estate. Do not change it
here.

## Verification

The included workflow is manual-only and calls the versioned shared workflow at
`IlleNilsson/.github@v1`.

# xmip-core-transport-iso-tp

ISO-TP transport: ISO 15765-2 over CAN — single, first, consecutive and flow-control frames segment a payload of up to 4095 bytes across the bus and reassemble it, block size and separation time from the receiver. A technology of [xmip-core-transport](https://github.com/IlleNilsson/xmip-core-transport).

`bus_of` reads the bus back out of an origin this technology wrote, `isotp://<bus>/0x<id>`, for UDS and OBD-II, which name the bus in their own origins; until 2026-09-28 each of them cut it out of the origin itself.

## Acknowledgement

Acceptance is at-most-once here. ISO 15765-2 acknowledges no message it
completed: flow control paces the sender before the data, and nothing answers
the last frame, so nobody is left to tell how the receive cycle ended. Each
message arrives whole. UDS, riding on this crate, answers its requests itself.

## The declaration

What a Location of this technology is configured with is two `const`s in its `declaration` module, `src/declaration.rs`, beside the code, which `Configured` reads (ADR-0064, amendment 2026-11-26): `declaration::SETTINGS`, `id`, `block_size`, `separation`, `escape` and `timeout`, and `SCHEMES`, `isotp`; the wait `timeout` defaults to, `declaration::DEFAULT_TIMEOUT`, is declared there too, with ISO 15765-4's physical pair, `TESTER_ID` and `ECU_ID`, which the UDS declaration defaults to; UDS and OBD-II take the wait, and UDS the pair, by this crate's own `declaration` feature, and the crate root names the three as before. Built with its `declaration` feature alone (`--no-default-features --features declaration`), the crate is those and nothing more: its code and every crate the code is built on — the can-bus technology it rides on among them — are behind the default feature, and `socketcan`, which is code, turns the default on with it. So the library the surfaces load can carry it without the code, for the language server and the desktop to suggest and check an ISO-TP Location's settings though no node in their process loaded the technology, once the gathering crate of open problem 48 links it. `test/Declaration.Test.ps1` in the estate holds that the declaration builds nothing the transport capability does not.

## Toolchain

`rust-toolchain.toml` pins the toolchain for the whole estate. Do not change it
here.

## Verification

The included workflow is manual-only and calls the versioned shared workflow at
`IlleNilsson/.github@v1`.

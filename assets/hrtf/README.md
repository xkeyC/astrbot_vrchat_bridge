# NOTICE: `steam-default-48k.bin`

The data file `steam-default-48k.bin` in this directory is not covered by the
repository's AGPL-3.0 license. It is licensed under the Apache License,
Version 2.0; the full text is in [`LICENSE-Apache-2.0`](LICENSE-Apache-2.0).

## What the file is

`steam-default-48k.bin` is a table of head-related impulse responses (HRIRs) in
this project's `VRCHRTF1` format (`crates/vrc-audio/src/table.rs`). The bridge
matches the game's binaural audio against it to tell which direction a voice
comes from (`docs/full-vr/speaker.md`).

It is derived from the default HRTF built into the Steam Audio SDK 4.8.1 by
Valve Corporation (https://github.com/ValveSoftware/steam-audio), which is
licensed under the Apache License, Version 2.0. The default HRTF's data is not
published as a file; it ships inside the SDK's `phonon` library.

## How it was derived (modifications)

`tools/hrtf-render` loaded the SDK's prebuilt `phonon` library (4.8.1, Windows
x64) and, for each direction of a grid, rendered a unit impulse through
`iplBinauralEffectApply` (`IPL_HRTFTYPE_DEFAULT`, 48 kHz, frame size 1024,
spatial blend 1, bilinear interpolation). The left and right outputs are that
direction's HRIRs. Then:

- **Sampled on a grid:** azimuth -180 to 178 degrees every 2 degrees,
  elevation -30 to 60 degrees every 10 degrees (1800 directions).
- **Cut to 256 taps:** every HRIR was cut to the same 256-sample window,
  starting at the earliest onset over all directions (the API's own latency
  before it is dropped).
- **Quantized to i16:** the samples are stored as 16-bit integers with one
  scale for the whole table. The API's per-ear peak delays are stored as f32.

No other part of the SDK is included in this repository.

## Copyright and license

```
Copyright 2017-2023 Valve Corporation.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```

The copyright line is the one in the SDK's `include/phonon.h`.

## CIPIC HRTF Database

The SDK's third-party notices (`THIRDPARTY.md`) list the CIPIC HRTF Database.
Whether Steam Audio's default HRTF is derived from CIPIC data is not known to
us. Its terms ask that every reproduction of any part of the materials include
the copyright notice, so the notice is reproduced here as a precaution:

```
Copyright (c) 2001 The Regents of the University of California. All Rights Reserved

THE REGENTS OF THE UNIVERSITY OF CALIFORNIA MAKE NO REPRESENTATION OR
WARRANTIES WITH RESPECT TO THE CONTENTS HEREOF AND SPECIFICALLY DISCLAIM ANY
IMPLIED WARRANTIES OR MERCHANTABILITY OR FITNESS FOR ANY PARTICULAR PURPOSE.
```

## No endorsement

This project is not affiliated with, sponsored by or endorsed by Valve
Corporation, VRChat Inc. or the Regents of the University of California. Their
names appear here only to state where the data comes from and under which terms.

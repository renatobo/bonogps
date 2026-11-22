## Reporting a vulnerability

If you find a security issue in BonoGPS, please [report it privately through GitHub](https://github.com/renatobo/
bonogps/security/advisories/new). This opens a private conversation with the maintainer. Please keep sensitive details
there while the report is being assessed.

Include whatever you know about the following:

- The firmware release or commit, ESP32 board, and build environment you used.
- Which connection or feature was involved, such as the WiFi configuration page, Bluetooth LE, Bluetooth Classic
(SPP), or over-the-air updates. Mention whether the device was running as a WiFi access point or had joined an
existing network, if relevant.
- What you observed, what access someone would need, and what they could read or change. A short description and a
redacted log from your own device are enough to start.

Remove WiFi passwords, pairing keys, and precise GPS coordinates from logs or screenshots before sharing them.

## Supported versions

Security fixes target the current code on `main`. For released firmware, start with the [latest BonoGPS release]
(https://github.com/renatobo/bonogps/releases/latest). Reports about older builds are welcome, especially if you
cannot safely upgrade. Backports to older releases are considered case by case.

## What happens next

The maintainer uses the private report to discuss impact, request any missing details, and share progress. BonoGPS is
community-maintained, so response times can vary. If there is a workaround, it can be shared in the report before a
firmware fix is available.

For a confirmed issue, the maintainer and reporter can coordinate when to publish the affected versions, mitigation,
and fixed release. If the report is closed without a fix, the maintainer will explain why in the private thread. Let
the maintainer know if you would like public credit or prefer to remain anonymous.

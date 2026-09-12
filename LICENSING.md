# Licensing

CogwareCan is dual licensed: GPL-3.0-only for open source and hobby use, or a
commercial license for anyone who cannot meet the GPL's terms. That covers
every crate here -- `cogware-can`, `cogware-canserver` and `cogware-canclient`.
The GPL text is in `LICENSE`.

## The open source license

Use, modify and study the code freely. The GPL's obligations attach only when
you *convey* a binary built from it — putting a board on your own car triggers
nothing at all, however far you modify the library.

If you do ship that firmware to someone else, GPL-3.0 §6 requires you to offer
them the complete corresponding source of the whole binary under the GPL, and,
because a CAN node in a vehicle is a User Product, the Installation Information
they need to reflash a modified build onto the device.

## The commercial license

The commercial license lifts both of those: you link `cogware-can` into closed
firmware, ship locked-down devices, and owe no source. It is the right option
if you are selling a product and the terms above are not compatible with how
you ship it.

Ask through <https://cogware.net/pages/contact>. Tell me what you are building
and how you distribute it; terms are per-product and negotiable.

## Copyright

Copyright (c) 2026 Justin Copenhaver. All contributions are licensed to the
copyright holder under the terms in `CONTRIBUTING.md`, which is what makes the
commercial license above possible.

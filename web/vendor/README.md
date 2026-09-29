# Vendored libraries

| File | Source | Version | Licence |
|---|---|---|---|
| `three.module.min.js` | `three/build/three.module.min.js` from the npm package `three` | 0.170.0 (r170) | MIT, Copyright 2010-2024 three.js authors |
| `OrbitControls.js` | `three/examples/jsm/controls/OrbitControls.js` from the same package | 0.170.0 (r170) | MIT, Copyright 2010-2024 three.js authors |
| `RoomEnvironment.js` | `three/examples/jsm/environments/RoomEnvironment.js` from the same package | 0.170.0 (r170) | MIT, Copyright 2010-2024 three.js authors |
| `RoundedBoxGeometry.js` | `three/examples/jsm/geometries/RoundedBoxGeometry.js` from the same package | 0.170.0 (r170) | MIT, Copyright 2010-2024 three.js authors |
| `BufferGeometryUtils.js` | `three/examples/jsm/utils/BufferGeometryUtils.js` from the same package | 0.170.0 (r170) | MIT, Copyright 2010-2024 three.js authors |

All files are unmodified copies. They are vendored so the viewer works
offline and without a build step; `web/index.html` maps the bare specifier
`three` to `./vendor/three.module.min.js` with an import map (the addons
import `three`).

To update: `npm pack three@<version>`, extract, and copy the files above.

Licence text (MIT):

    The MIT License

    Copyright © 2010-2024 three.js authors

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.

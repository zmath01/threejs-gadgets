# Three.js Gadgets

A collection of interactive 3D demos built with [Three.js](https://threejs.org/), deployed live via GitHub Pages.

## Live site

https://zmath01.github.io/threejs-gadgets

## Demos

| Demo | Description | File |
| --- | --- | --- |
| ⚙️ Differential Simulation | Interactive gear differential & torque-feature visualizer | [`differential-01.html`](differential-01.html) |
| 🏎️ F1 Systems Lab | Power unit & dynamics visualizer with engine/gearbox telemetry | [`f1-system-full.html`](f1-system-full.html) |
| 🧊 Mental Rotation Trainer | Allocentric spatial trainer with rotation/timing scoring | [`mental-rotation-test.html`](mental-rotation-test.html) |
| ✨ Compute Showcase | Matrix math, particle physics and Game of Life computed in JavaScript and rendered in WebGL | [`matrix-particles-game.html`](matrix-particles-game.html) |

| 🚁 Drone Simulation | Flight path, state estimation, and control prediction | [`drone-simulation.html`](drone-simulation.html) |

## Tech stack

- **Three.js** — loaded from CDN (jsdelivr / unpkg / cdnjs), no build step or local dependencies
- **GitHub Pages + GitHub Actions** — automatic deploy on push to `main`

## Development

The demos are self-contained static HTML files. Open any `.html` file directly in a browser — no server or install required.

## Deploy

Everything is deployed automatically by the workflow in [`.github/workflows/deploy.yml`](.github/workflows/deploy.yml) whenever you push to `main`.

1. Push to the `main` branch.
2. In the repo go to **Settings → Pages** and set **Build and deployment → Source → GitHub Actions**.
3. The latest push is live at `https://zmath01.github.io/threejs-gadgets`.

## License

[MIT](LICENSE) © 2026 zmath01
<img src="docs/logo.svg" alt="Newton logo" width="96">

# Newton

**A 2D physics engine in Rust, built from scratch.**

Newton simulates circular rigid bodies under gravity. It integrates motion with semi-implicit Euler, detects circle-circle and circle-wall collisions, and resolves them with impulse-based physics that accounts for mass and restitution, followed by a positional correction so bodies do not sink into each other or the floor. There is no physics crate and no math crate underneath it, the vector math and the solver are plain Rust you can read top to bottom.

## What it is

- Circular rigid bodies with position, velocity, mass, and restitution
- Semi-implicit (symplectic) Euler integration under gravity
- Circle-circle collision detection and impulse-based resolution
- Circle-versus-wall collisions against a bounding box
- Positional correction to stop bodies from sinking into each other
- A fixed-timestep `World` you step forward frame by frame
- Deterministic and stable: no NaNs, no panics, bodies at rest actually come to rest

## Usage

Add bodies to a `World` and step it forward:

```rust
use newton::{Body, Bounds, Vec2, World};

let bounds = Bounds { min: Vec2::new(0.0, 0.0), max: Vec2::new(20.0, 20.0) };
let mut world = World::new(Vec2::new(0.0, -9.8), bounds);

world.add_body(Body::new(Vec2::new(10.0, 18.0), 1.0, 1.0, 0.6));

for _ in 0..300 {
    world.step(1.0 / 60.0);
}
```

Or run the CLI, which sets up a small demo scene and prints where everything ends up:

```
cargo run -- run --steps 300
```

## Tests

```
cargo test
```

The suite checks real physical behavior, not just that the code compiles: a body under gravity falls, a dropped body rests above the floor instead of tunneling through it, a head-on elastic collision between equal masses exchanges velocity, momentum is conserved across a collision, and restitution under 1.0 loses energy on every bounce.

## Try it in the browser

`docs/index.html` runs a JavaScript port of the same integrator and collision resolver on a canvas. Click to drop a ball, adjust gravity and restitution, and watch the same math run live.

By Pavan Nallamothu.

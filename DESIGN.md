# Design

Newton is small on purpose. Three pieces do all the work: integration, collision detection, and impulse resolution. This document walks through each one and why it is built the way it is.

## Integration

Every body carries a position, a velocity, a mass, and a restitution. Each step applies gravity and moves the body with **semi-implicit (symplectic) Euler** integration:

```
velocity += gravity * dt
position += velocity * dt
```

The key detail is the order. Explicit Euler would update position using the *old* velocity, before gravity has been applied for this step. Semi-implicit Euler updates velocity first and then uses the new velocity to move position. That single change makes the integrator far more stable under constant acceleration like gravity, at effectively no extra cost. It is why bodies settle into a resting state instead of slowly gaining energy over thousands of steps.

Static bodies, meaning walls or anything with zero inverse mass, are skipped during integration entirely. They never move regardless of what collides with them.

## Collision detection

Two shapes of check happen every step, always after integration so detection sees where bodies actually ended up this frame.

**Circle versus circle.** Two circles overlap when the distance between their centers is less than the sum of their radii. That is the entire test, one subtraction, one length, one comparison. No broad phase is needed at the small body counts this engine targets, so every pair is checked directly.

**Circle versus wall.** The world has a rectangular bounding box. A body penetrates a wall when its center, offset by its radius, crosses one of the four edges. Each axis is checked independently, so a body can be resolved against a side wall and the floor in the same step without special-casing corners.

## Impulse resolution

When two bodies overlap, the collision normal is the unit vector between their centers, pointing from one to the other. The relative velocity along that normal decides what happens next:

- If the bodies are already separating along the normal, no impulse is applied. Applying one anyway would add energy that was never there, which is a classic way to make a physics engine explode.
- Otherwise, an impulse is computed from the standard restitution formula, split between the two bodies by their inverse mass, so a heavier body moves less and a zero-inverse-mass wall does not move at all:

```
impulse = -(1 + restitution) * (relative_velocity . normal) / (invMassA + invMassB)
```

That impulse is added to one body's velocity and subtracted from the other's, along the normal. Restitution of 1.0 is a fully elastic bounce, and any value below 1.0 removes energy from the pair on every contact, which is what makes a dropped ball's bounces get smaller over time instead of staying the same height forever.

## Positional correction

Impulse resolution alone fixes velocity but not position. Two circles that are overlapping when a collision is detected would otherwise keep overlapping, or slowly sink further into each other under gravity even as their velocities look correct, since the velocity solve only prevents *further* approach, not the overlap that already exists.

To fix that, after the velocity impulse, Newton nudges the two bodies apart directly along the normal, proportional to how much they overlap and split by inverse mass, same as the velocity impulse:

```
correction = max(penetration - slop, 0) / (invMassA + invMassB) * correction_percent
```

Two constants keep this from causing its own problems:

- **`slop`** ignores a tiny sliver of overlap. Without it, the corrector would fight to remove the last fraction of a millimeter forever, which shows up as visible jitter in a body that should be at rest.
- **`correction_percent`** is less than 1.0, so only part of the overlap is corrected each step rather than all of it at once. Correcting everything in a single step tends to overshoot and bounce, correcting a fraction converges smoothly over a few frames instead.

Together, the three pieces cover the whole loop a rigid-body engine needs: bodies move under a stable integrator, contacts are found with plain geometry, and contacts are resolved so that velocity and position both end up physically consistent, without ever needing a physics or math crate to get there.

use crate::body::Body;
use crate::vec2::Vec2;

/// Fraction of overlap corrected per step. Less than 1.0 so correction does
/// not itself inject energy into the system.
const POSITION_CORRECTION_PERCENT: f64 = 0.8;
/// Overlap below this is left alone, so resting bodies do not jitter as the
/// solver chases a last fraction of a millimeter forever.
const POSITION_SLOP: f64 = 0.01;
/// Below this relative speed a bounce is treated as a rest contact instead
/// of a collision, so gravity does not fight restitution into endless jitter.
const REST_VELOCITY_THRESHOLD: f64 = 0.5;

/// An axis-aligned box that bodies collide against from the inside.
#[derive(Debug, Clone, Copy)]
pub struct Bounds {
    pub min: Vec2,
    pub max: Vec2,
}

pub struct World {
    pub bodies: Vec<Body>,
    pub gravity: Vec2,
    pub bounds: Bounds,
}

impl World {
    pub fn new(gravity: Vec2, bounds: Bounds) -> Self {
        World {
            bodies: Vec::new(),
            gravity,
            bounds,
        }
    }

    pub fn add_body(&mut self, body: Body) -> usize {
        self.bodies.push(body);
        self.bodies.len() - 1
    }

    /// Advances the simulation by one fixed timestep: integrate, then
    /// resolve wall collisions, then resolve body-body collisions.
    pub fn step(&mut self, dt: f64) {
        self.integrate(dt);
        self.resolve_walls();
        self.resolve_body_collisions();
    }

    fn integrate(&mut self, dt: f64) {
        for body in &mut self.bodies {
            if body.is_static() {
                continue;
            }
            // Semi-implicit (symplectic) Euler: update velocity from
            // acceleration first, then use the *new* velocity to move the
            // position. More stable under gravity than explicit Euler.
            body.velocity += self.gravity * dt;
            body.position += body.velocity * dt;
        }
    }

    fn resolve_walls(&mut self) {
        let bounds = self.bounds;
        for body in &mut self.bodies {
            if body.is_static() {
                continue;
            }

            if body.position.x - body.radius < bounds.min.x {
                body.position.x = bounds.min.x + body.radius;
                if body.velocity.x < 0.0 {
                    body.velocity.x = -body.velocity.x * body.restitution;
                }
            } else if body.position.x + body.radius > bounds.max.x {
                body.position.x = bounds.max.x - body.radius;
                if body.velocity.x > 0.0 {
                    body.velocity.x = -body.velocity.x * body.restitution;
                }
            }

            if body.position.y - body.radius < bounds.min.y {
                body.position.y = bounds.min.y + body.radius;
                if body.velocity.y < 0.0 {
                    body.velocity.y = -body.velocity.y * body.restitution;
                }
                if body.velocity.y.abs() < REST_VELOCITY_THRESHOLD {
                    body.velocity.y = 0.0;
                }
            } else if body.position.y + body.radius > bounds.max.y {
                body.position.y = bounds.max.y - body.radius;
                if body.velocity.y > 0.0 {
                    body.velocity.y = -body.velocity.y * body.restitution;
                }
            }
        }
    }

    fn resolve_body_collisions(&mut self) {
        let count = self.bodies.len();
        for i in 0..count {
            for j in (i + 1)..count {
                self.resolve_pair(i, j);
            }
        }
    }

    fn resolve_pair(&mut self, i: usize, j: usize) {
        let (a, b) = (self.bodies[i], self.bodies[j]);
        if a.is_static() && b.is_static() {
            return;
        }

        let delta = b.position - a.position;
        let dist = delta.length();
        let min_dist = a.radius + b.radius;
        if dist >= min_dist {
            return;
        }

        // Bodies started exactly on top of each other. Pick an arbitrary
        // normal rather than dividing by a zero-length delta.
        let normal = if dist > 1e-9 {
            delta / dist
        } else {
            Vec2::new(1.0, 0.0)
        };
        let penetration = min_dist - dist;

        let relative_velocity = b.velocity - a.velocity;
        let velocity_along_normal = relative_velocity.dot(normal);

        // Bodies are overlapping but separating (or resting): still push
        // them apart positionally, but do not apply an impulse that would
        // add energy on every subsequent frame of contact.
        if velocity_along_normal <= 0.0 {
            let restitution = a.restitution.min(b.restitution);
            let inv_mass_sum = a.inv_mass + b.inv_mass;
            if inv_mass_sum > 0.0 {
                let impulse_mag = -(1.0 + restitution) * velocity_along_normal / inv_mass_sum;
                let impulse = normal * impulse_mag;
                self.bodies[i].velocity -= impulse * a.inv_mass;
                self.bodies[j].velocity += impulse * b.inv_mass;
            }
        }

        // Positional correction: push the pair apart along the normal,
        // split by inverse mass, so a static wall never moves.
        let inv_mass_sum = a.inv_mass + b.inv_mass;
        if inv_mass_sum > 0.0 {
            let correction_mag =
                (penetration - POSITION_SLOP).max(0.0) / inv_mass_sum * POSITION_CORRECTION_PERCENT;
            let correction = normal * correction_mag;
            self.bodies[i].position -= correction * a.inv_mass;
            self.bodies[j].position += correction * b.inv_mass;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_bounds() -> Bounds {
        Bounds {
            min: Vec2::new(0.0, 0.0),
            max: Vec2::new(100.0, 100.0),
        }
    }

    #[test]
    fn gravity_pulls_body_down() {
        let mut world = World::new(Vec2::new(0.0, -9.8), default_bounds());
        world.add_body(Body::new(Vec2::new(50.0, 50.0), 1.0, 1.0, 0.5));

        let start_y = world.bodies[0].position.y;
        for _ in 0..10 {
            world.step(1.0 / 60.0);
        }

        assert!(world.bodies[0].velocity.y < 0.0, "body should gain downward velocity");
        assert!(world.bodies[0].position.y < start_y, "body should fall");
    }

    #[test]
    fn body_settles_above_floor_without_tunneling() {
        let mut world = World::new(Vec2::new(0.0, -30.0), default_bounds());
        world.add_body(Body::new(Vec2::new(50.0, 90.0), 2.0, 1.0, 0.1));

        for _ in 0..600 {
            world.step(1.0 / 60.0);
        }

        let body = world.bodies[0];
        assert!(body.position.y >= body.radius - 1e-6, "body must not tunnel through the floor");
        assert!(
            (body.position.y - body.radius).abs() < 0.5,
            "body should come to rest close to the floor, got y={}",
            body.position.y
        );
        assert!(body.velocity.y.abs() < 1.0, "resting body should not still be bouncing");
    }

    #[test]
    fn elastic_head_on_collision_exchanges_velocity() {
        // No gravity, no walls in the way: an isolated exchange.
        let mut world = World::new(
            Vec2::ZERO,
            Bounds {
                min: Vec2::new(-1000.0, -1000.0),
                max: Vec2::new(1000.0, 1000.0),
            },
        );
        world.add_body(Body::new(Vec2::new(0.0, 0.0), 1.0, 1.0, 1.0).with_velocity(Vec2::new(5.0, 0.0)));
        world.add_body(Body::new(Vec2::new(1.99, 0.0), 1.0, 1.0, 1.0));

        world.step(1.0 / 60.0);

        let a = world.bodies[0];
        let b = world.bodies[1];
        assert!(a.velocity.x.abs() < 0.5, "moving body should have handed off its velocity, got {}", a.velocity.x);
        assert!((b.velocity.x - 5.0).abs() < 0.5, "struck body should now carry the velocity, got {}", b.velocity.x);
    }

    #[test]
    fn momentum_is_conserved_across_collision() {
        let mut world = World::new(
            Vec2::ZERO,
            Bounds {
                min: Vec2::new(-1000.0, -1000.0),
                max: Vec2::new(1000.0, 1000.0),
            },
        );
        world.add_body(Body::new(Vec2::new(0.0, 0.0), 1.0, 2.0, 0.8).with_velocity(Vec2::new(4.0, 0.0)));
        world.add_body(Body::new(Vec2::new(1.99, 0.0), 1.0, 3.0, 0.8).with_velocity(Vec2::new(-1.0, 0.0)));

        let momentum_before: f64 = world
            .bodies
            .iter()
            .map(|b| b.mass * b.velocity.x)
            .sum();

        world.step(1.0 / 60.0);

        let momentum_after: f64 = world
            .bodies
            .iter()
            .map(|b| b.mass * b.velocity.x)
            .sum();

        assert!(
            (momentum_before - momentum_after).abs() < 1e-6,
            "momentum should be conserved: before={}, after={}",
            momentum_before,
            momentum_after
        );
    }

    #[test]
    fn restitution_below_one_loses_energy_on_bounce() {
        let mut world = World::new(Vec2::new(0.0, -20.0), default_bounds());
        world.add_body(Body::new(Vec2::new(50.0, 20.0), 1.0, 1.0, 0.5));

        // Step until the body has fallen and bounced back upward off the floor once.
        let mut peak_before_bounce: f64 = 0.0;
        let mut bounced = false;
        let mut peak_after_bounce: f64 = 0.0;

        for _ in 0..300 {
            let prev_vy = world.bodies[0].velocity.y;
            world.step(1.0 / 60.0);
            let vy = world.bodies[0].velocity.y;

            if !bounced && prev_vy < 0.0 && vy > 0.0 {
                bounced = true;
                peak_before_bounce = 20.0 - 1.0; // starting height above floor contact point
            }
            if bounced {
                peak_after_bounce = peak_after_bounce.max(world.bodies[0].position.y - 1.0);
            }
        }

        assert!(bounced, "body should have bounced off the floor");
        assert!(
            peak_after_bounce < peak_before_bounce,
            "restitution < 1 should lose height/energy each bounce: before={}, after={}",
            peak_before_bounce,
            peak_after_bounce
        );
    }
}

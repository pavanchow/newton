use crate::vec2::Vec2;

/// A circular rigid body.
#[derive(Debug, Clone, Copy)]
pub struct Body {
    pub position: Vec2,
    pub velocity: Vec2,
    pub radius: f64,
    pub mass: f64,
    /// 0 for an infinite-mass body such as a wall. Precomputed so collision
    /// resolution never divides by mass directly.
    pub inv_mass: f64,
    /// 0.0 loses all relative velocity on impact, 1.0 is a perfectly elastic bounce.
    pub restitution: f64,
}

impl Body {
    pub fn new(position: Vec2, radius: f64, mass: f64, restitution: f64) -> Self {
        let inv_mass = if mass > 0.0 { 1.0 / mass } else { 0.0 };
        Body {
            position,
            velocity: Vec2::ZERO,
            radius,
            mass,
            inv_mass,
            restitution,
        }
    }

    pub fn with_velocity(mut self, velocity: Vec2) -> Self {
        self.velocity = velocity;
        self
    }

    pub fn is_static(&self) -> bool {
        self.inv_mass == 0.0
    }
}
